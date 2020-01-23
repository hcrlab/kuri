# kuri_api

Provides Python wrappers for hardware capabilities and some high level functionality 

* Animation library and utilities
* mobile base
* chest lights
* volume management
* speech and speech recognition

## Usage

### Dependencies

You may need to install the following packages:
- `sudo pip install timer`
- `sudo pip install pyttsx`

### Simulation

You may need to install `alsaaudio` on your machine

    sudo pip install pyalsaudio
    
Some utilities expect Kuri's animation assets to be in place. You can mock this up by copying `/opt/gizmo/share/assets` off the robot onto your machine at the same location. Make sure your user has write access to this directory as well. 
