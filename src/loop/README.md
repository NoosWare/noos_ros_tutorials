# Loop tutorial

Loop is a basic example about how to use Noos API in a simple loop
using the same object, but changing the content which is sent to the 
cloud platform.

The example sends an image every second to detect a face in the image.
An usb camera can be used to check the results, otherwise the same
image will be sent. The image can be found in `/data` folder as `lenna.png`.

To run it:

```shell
rosrun loop loop_node
```
