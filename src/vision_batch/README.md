## Vision Batch tutorial

Vision Batch example shows how to use the class `vision_batch` of the 
Noos API.

The example sends an image to the cloud platform every second to detect 
a face in the image. If a face is found, a second call is done to the platform,
but in this case, the image is cropped to send only the face for recogniting
the face expression and the age.

An usb camera can be used to check the results, otherwise the same
image will be sent. The image can be found in `/data` folder as `lenna.png`.

To run it:

```shell
rosrun vision_batch vision_batch_node
```
