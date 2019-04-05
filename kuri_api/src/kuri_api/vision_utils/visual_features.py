from copy import deepcopy
from vision_msgs.msg import Face, ClassifiedObject
import numpy as np
from collections import OrderedDict
from .utils import cast_nan, get_key_ref_by_path, set_key_by_path
import logging
logger = logging.getLogger(__name__)

class VisualFeatures:

    def __init__(self, fsps=None, msps=None, features=None):
        self.config = self.Config(fsps=None, msps=None, features=None)
        self.calc = self.Calc(config=self.config)
        return

    class Config():
        """
        Generate configs and empty data structures for frame and moment feats
        
        configs are nested dicts of features -> indices in the data structure
        the data structure is a numpy array of values to allow more efficient
        calculation of statistics.
        
        If you wish to alter a config or data structure, use the get_* methods
        as these will return a copy rather than a reference.
        
        data can be rendered into the config dict using render
        """

        def __init__(self, fsps=None, msps=None, features=None):
            """
            :param fsps: patterns for frame stat configs
            :param msps: patterns for moment stat configs
            :param features: list of features and stat patterns upon them
            :returns (frame conf, frame data, moment conf, moment data)
            """
            self.fsps = fsps or {'sv': {'labels': ('value', ), 'indices': (1, )}, 
               'mi': {'labels': ('count', 'mean', 'variance'), 'indices': (0, 1, 2)}}
            frame_stat_len = 0
            for _fps_k, _fsp_v in self.fsps.iteritems():
                _t = np.max(_fsp_v['indices']) + 1
                if _t > frame_stat_len:
                    frame_stat_len = _t

            self.msps = msps or {'mv': {'labels': ('mean', 'var'), 'indices': (0, 1)}}
            moment_stat_len = 0
            for _msp_k, _msp_v in self.msps.iteritems():
                _t = np.max(_msp_v['indices']) + 1
                if _t > moment_stat_len:
                    moment_stat_len = _t

            self.features = features or [
             (('det', 'face', 'extent'), 'mi', 'mv', 'face_detector'),
             (('det', 'face', 'comp'), 'mi', 'mv', 'face_detector'),
             (('det', 'cat', 'extent'), 'mi', 'mv', 'object_detector'),
             (('det', 'dog', 'extent'), 'mi', 'mv', 'object_detector'),
             (('det', 'person', 'extent'), 'mi', 'mv', 'object_detector'),
             (('excitement',), 'sv', 'mv', 'object_detector'),
             (('brightness',), 'sv', 'mv', 'image_quality'),
             (('sharpness',), 'sv', 'mv', 'image_quality'),
             (('score',), 'sv', 'mv', '')]
            self.feat_modules = list(set([ f[3] for f in self.features ]))
            feat_len = len(self.features)
            self.fmods = OrderedDict()
            for index, f in enumerate(self.features):
                mod = f[3]
                if mod not in self.fmods:
                    self.fmods[mod] = []
                self.fmods[mod].append(index)

            moment_stat_len = frame_stat_len * moment_stat_len
            self.ff_data = np.zeros((feat_len, frame_stat_len))
            self.ff_data.fill(np.nan)
            self.ff = {}
            for _feat_index, _feat in enumerate(self.features):
                _cur_loc = self.ff
                for _label in _feat[0]:
                    if _label not in _cur_loc:
                        _cur_loc[_label] = {}
                    _cur_loc = _cur_loc[_label]

                _fsp = self.fsps[_feat[1]]
                for _fsp_index, _fsp_label in enumerate(_fsp['labels']):
                    _cur_loc[_fsp_label] = (
                     _feat_index,
                     _fsp['indices'][_fsp_index])

            self.mf_data = np.zeros((feat_len, moment_stat_len))
            self.mf_data.fill(np.nan)
            self.mf = {}
            for _feat_index, _feat in enumerate(self.features):
                _cur_loc = self.mf
                for _label in _feat[0]:
                    if _label not in _cur_loc:
                        _cur_loc[_label] = {}
                    _cur_loc = _cur_loc[_label]

                _msp = self.msps[_feat[2]]
                _fsp = self.fsps[_feat[1]]
                for _msp_index, _msp_label in enumerate(_msp['labels']):
                    if _msp_label not in _cur_loc:
                        _cur_loc[_msp_label] = {}
                    for _fsp_index, _fsp_label in enumerate(_fsp['labels']):
                        _s_index = _fsp['indices'][_fsp_index] + _msp_index * frame_stat_len
                        _cur_loc[_msp_label][_fsp_label] = (_feat_index,
                         _s_index)

        def get_ff_ind(self, key_path):
            return get_key_ref_by_path(self.ff, key_path)

        def get_mf_ind(self, key_path):
            return get_key_ref_by_path(self.mf, key_path)

        def get_ff_data(self):
            return np.copy(self.ff_data)

        def get_mf_data(self):
            return np.copy(self.mf_data)

    class Calc():
        """
        Logic and methods for calculating features. Uses config to decide
        calculations to perform and population into the data structure
        
        TODO(Joe): Better placement for params
        """
        OBJECTS_HEIGHT_SCALER = 1.0 / 300.0
        OBJECTS_WIDTH_SCALER = 1.0 / 300.0
        OBJECTS_SCALER = OBJECTS_HEIGHT_SCALER * OBJECTS_WIDTH_SCALER
        POINTS_OF_INTEREST = [
         (0.5, 0.5), (0.33, 0.33), (0.66, 0.33)]

        def __init__(self, config):
            self.config = config

        def comp(self, item):
            """
            Composition, currently minimum distance to a point of interest,
            currently upper law of thirds and center point.
            
            TODO(Joe): Add presence along edge measure
            """
            poi = self.POINTS_OF_INTEREST
            coords = ()
            if isinstance(item, Face):
                coords = (
                 item.center.x, item.center.y)
            else:
                if isinstance(item, ClassifiedObject):
                    coords = (
                     (item.roi.x_offset + item.roi.width / 2.0) * self.OBJECTS_WIDTH_SCALER,
                     (item.roi.y_offset + item.roi.height / 2.0) * self.OBJECTS_HEIGHT_SCALER)
            return np.min(np.linalg.norm(np.subtract(poi, coords), axis=1))

        def extent(self, item):
            if isinstance(item, Face):
                return item.size
            if isinstance(item, ClassifiedObject):
                return item.roi.width * item.roi.height * self.OBJECTS_SCALER

        def det_face(self, ff, op):
            fun = getattr(self, op)
            return [ fun(face) for face in ff['msg'].faces.faces ]

        def _det_obj(self, ff, op, oc):
            fun = getattr(self, op)
            return [ fun(o) for o in ff['msg'].objects.positive_detections.objects if o.object_class == oc
                   ]

        def det_cat(self, ff, op):
            return self._det_obj(ff, op, 'cat')

        def det_dog(self, ff, op):
            return self._det_obj(ff, op, 'dog')

        def det_person(self, ff, op):
            return self._det_obj(ff, op, 'person')

        def sharpness(self, ff):
            return ff['msg'].quality.sharpness

        def brightness(self, ff):
            return ff['msg'].quality.brightness

        def excitement(self, ff):
            ex = np.nan
            for o in ff['msg'].objects.all_detections.objects:
                if o.object_class == 'Excitement':
                    ex = o.confidence

            return ex

        def score(self, ff):
            face_extent = self.get_f_val(['det', 'face', 'extent', 'mean'], ff['data']) or 0.0
            face_count = self.get_f_val(['det', 'face', 'extent', 'count'], ff['data']) or 0.0
            excitement = self.get_f_val(['excitement', 'value'], ff['data']) or 0.0
            sharpness = self.get_f_val(['sharpness', 'value'], ff['data']) or 0.0
            return np.nansum([face_extent * face_count,
             sharpness / 4.0,
             excitement / 20.0])

        def get_f_val(self, key_path, data):
            k = get_key_ref_by_path(self.config.ff, key_path)
            if k is not None:
                return data[k]
            return
            return

        def f_stats(self, feature, fsps_def):
            """
            Generate stats for frame feature
            
            Must follow pattern in config.fsps
            
            TODO(Joe): Set length and organize stats from fsps_def
            """
            if fsps_def == 'mi':
                if len(feature):
                    return (len(feature), np.sum(feature), np.var(feature))
            else:
                if fsps_def == 'sv':
                    if feature is not None:
                        return (np.nan, feature, np.nan)
            return (0.0, 0.0, 0.0)

        def populate_frame_features(self, ff, feature_indices):
            for f_ind in feature_indices:
                f = self.config.features[f_ind]
                items = None
                if len(f[0]) > 1:
                    fun = getattr(self, ('_').join(f[0][:-1]))
                    items = fun(ff, f[0][-1])
                else:
                    if len(f[0]) == 1:
                        fun = getattr(self, f[0][0])
                        items = fun(ff)
                ff['data'][f_ind] = self.f_stats(items, f[1])

            return

        def featurize_frame(self, msg):
            ff = {'msg': msg, 'data': self.config.get_ff_data()}
            for mod, f_inds in self.config.fmods.iteritems():
                if mod == '' or mod in ff['msg'].modules:
                    self.populate_frame_features(ff, f_inds)

            return ff

        def gen_moment_data(self, ffs, m_start, m_stop):
            """
            Integration of frame results for whole moment representation
            
            Generates values for "area" of results ~(value X time)
            
            Timeline:
            
            |_[__|____|_|__|__|___|____|_|____|___]__|
            
            |       Featurized frame (run through vision, not the moment video)
            [ ]     Moment video first and last frame timestamps
            
            We pad a result before and after the moment.
            
            m_start must be between ffs[0], ffs[1],
            m_stop must be between ffs[-2], ffs[-1]
            
            :param ffs: Featurized frames to integrate. (oldest to newest)
            :param m_start: time stamp of first frame in moment
            :param m_stop: time stamp of last frame in moment
            :returns moment data array
            """
            mf_data = self.config.get_mf_data()
            if len(ffs) < 5:
                return mf_data

            def weight_time(m_start, m_stop, ff_times):
                r"""
                Weight time based on mid point between frame and
                adjacent frames.
                
                |___.____|______.______|____.____|_._|
                    \___________/\__________/\_____/
                
                |       featurized frame
                .       mid point between frames
                \_/     time area assigned to contained frame
                
                Generate time weights for featurized frames.
                Assumes 1 ff_time before and after moment
                Must be at least 5 featurized frames
                
                :param m_start float epoch timestamp of first frame in moment
                :param m_stop float epoch timestamp of last frame in moment
                :param ff_times list of float epoch times for detected frames
                :param ff_feat_mask mask of frames vs features present in
                :returns np.array((len(ff_times), 1)) of time based weights
                """
                ff_t = np.array(ff_times)
                ff_weights = np.zeros(ff_t.shape)
                first_mid = (ff_t[1] - ff_t[0]) / 2.0 + ff_t[0]
                first_edge = m_start if first_mid < m_start else first_mid
                if first_mid < m_start:
                    first_edge = m_start
                    ff_weights[0] = 0.0
                else:
                    first_edge = first_mid
                    ff_weights[0] = first_mid - m_start
                ff_weights[1] = (ff_t[2] - ff_t[1]) / 2.0 + (ff_t[1] - first_edge)
                last_mid = (ff_t[-1] - ff_t[-2]) / 2.0 + ff_t[-2]
                last_edge = m_stop if last_mid > m_stop else last_mid
                if last_mid > m_stop:
                    last_edge = m_stop
                    ff_weights[-1] = 0.0
                else:
                    last_edge = last_mid
                    ff_weights[-1] = m_stop - last_mid
                ff_weights[-2] = (ff_t[-2] - ff_t[-3]) / 2.0 + (last_edge - ff_t[-2])
                ff_weights[2:(-2)] = ((ff_t[2:] - ff_t[:-2]) / 2.0)[1:-1]
                return ff_weights

            def moment_stats(ffs):
                """
                Generate moment level statistics for frame level features
                
                Use weight_time for time weighted stats (mean, variance...)
                
                :param ffs: Featurized frames
                :returns: Moment level statistics data array
                """
                frames_data = np.array([ ff['data'] for ff in ffs ])
                frame_times = [ ff['msg'].header.stamp.to_sec() for ff in ffs ]
                frames_time_weights = weight_time(m_start, m_stop, frame_times)
                frame_features = np.nansum(frames_data, axis=2)
                frame_features_mask = np.isfinite(frame_features)
                frame_features_weights = np.expand_dims(frames_time_weights, axis=1) * frame_features_mask
                feature_weight_sums = np.sum(frame_features_weights, axis=0)
                feature_weight_sums = np.expand_dims(feature_weight_sums, axis=1)
                weighted_frames = np.einsum('ijk,i->ijk', frames_data, frames_time_weights)
                weighted_sum_of_frames = np.nansum(weighted_frames, axis=0)
                time_weighted_mean_features = weighted_sum_of_frames / feature_weight_sums
                demeaned_frames_data = frames_data - time_weighted_mean_features
                demeaned_weighted_frames = np.einsum('ijk,i->ijk', demeaned_frames_data ** 2, frames_time_weights)
                demeaned_weighted_sum_of_frames = np.nansum(demeaned_weighted_frames, axis=0)
                time_weighted_variance_features = demeaned_weighted_sum_of_frames / feature_weight_sums
                mv_feature_stats = np.concatenate((time_weighted_mean_features,
                 time_weighted_variance_features), axis=1)
                return mv_feature_stats

            mf_data = moment_stats(ffs)
            return mf_data

    def render_ff(self, data, cast_nan_to_none=False):
        """
        Render data into a copy of the ff (frame features) config
        """
        if data.shape == self.config.ff_data.shape:
            return self.render(self.config.ff, data, cast_nan_to_none)

    def render_mf(self, data, cast_nan_to_none=False):
        """
        Render data into a copy of the mf (moment features) config
        """
        if data.shape == self.config.mf_data.shape:
            return self.render(self.config.mf, data, cast_nan_to_none)

    def render_f_result(self, result, cast_nan_to_none=False):
        """
        """
        result['stats'] = self.render_ff(result['data'], cast_nan_to_none)
        return result

    def render(self, conf, data, cast_nan_to_none=False):
        """
        Render a feature config nested dict with data
        
        :param conf: conf produced by gen_visual_features_config()
        :param data: data struct produced by gen_visual_features_config()
        """
        output = deepcopy(conf)

        def populate_config(output, d, key_path):
            if isinstance(d, dict):
                for k in d:
                    populate_config(output, d[k], key_path + [k])

            else:
                if isinstance(d, tuple):
                    if cast_nan_to_none:
                        set_key_by_path(output, key_path, cast_nan(data[d], None))
                    else:
                        set_key_by_path(output, key_path, data[d])
            return

        populate_config(output, output, [])
        return output