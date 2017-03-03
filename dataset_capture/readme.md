### Compiling ###

inside /localization_and_tracking/dataset_capture run
`mkdir build && cd build && cmake ..`

### Running ###
* after compiling a bin folder will be created (/localizing_and_tracking/dataset_capture/bin). Go inside it and run
`./dataset_capture "#1" -dataset <path-to-localizing_and_tracking>/dataset/close_directional_light -model3d <path-to-localizing_and_tracking>/dataset/turtle/turtle
`
* OBS: even though this code runs over the dataset, it is necessary to have a connected kinnect. The reason for that is that I didn't bothered decoupling the kinect streaming code inside the "openni_capture.h" file. Improvements on this are welcome!
