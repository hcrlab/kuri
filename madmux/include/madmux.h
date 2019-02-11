#ifndef MADMUX_H
#define MADMUX_H

#include <stdint.h> /* uint* */

#ifdef __cplusplus
extern "C" {
#endif

struct mdx_stream;

typedef void (*mdx_cb)(uint8_t* buffer, uint32_t size, void* user_data);

/**
 * Open a madmux video channel
 * @param  socket the path to the UNIX socket for that channel
 * @return        the video stream handle
 */
struct mdx_stream* mdx_open(const char* socket);

/**
 * Close a madmux video channel
 * @param stream the stream to close
 */
void mdx_close(struct mdx_stream* stream);

/**
 * Register a callback to receive video frames
 * @param stream    the video stream handle
 * @param cb        the callback
 * @param user_data a pointer to a user data structure
 */
void mdx_register_cb(struct mdx_stream* stream, mdx_cb cb, void* user_data);

/**
 * Force an i-frame in an h264 stream
 * @param stream    the video stream handle
 */
void mdx_force_iframe(struct mdx_stream* stream);

/**
 * Set the bitrate of a non-raw stream.
 * @param stream    the video stream handle
 * @param bitrate   bitrate in bps
 */
void mdx_set_bitrate(struct mdx_stream* stream, uint32_t bitrate);

/**
 * Set the resolution of a stream.
 * @param stream    the video stream handle
 * @param width     the width of the video in pixels
 * @param height    the height of the video in pixels
 */
void mdx_set_resolution(struct mdx_stream* stream, uint16_t width,
        uint16_t height);

#ifdef __cplusplus
}
#endif

#endif /* end of include guard: MADMUX_H */
