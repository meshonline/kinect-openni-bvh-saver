![](mocap.png)
# kinect-openni-bvh-saver
### Note: If you want an enhanced version for Windows platform, go to [http://www.mesh-online.net/mocap.html](http://www.mesh-online.net/mocap.html), the enhanced version can use multiple Kinect sensors for Xbox 360, thus captures true 360-degree range of motions in realtime.<br/>
<br/>
This project is based on OpenNI2, NITE2, and OpenCV2, it will automatically save many skeleton animation clips to bvh files.<br/>
<br/>
Then you can import the bvh files into Blender or other softwares to smooth animation curves. I use Blender's default smooth key setting to smooth all rotation channels, the results is acceptable.<br/>
<br/>
You can turn your body around 360 degrees in this project, I am very satisfied with the result.<br/>
<br/>
Though the project is developed on macOS, you can port it to Linux, Windows easily, for the source code is portable.<br/>
<br/>

Download windows release from: [https://drive.google.com/file/d/0B37dehusTo2MTWJEdkdrd29pLWM/view?usp=sharing](https://drive.google.com/file/d/0B37dehusTo2MTWJEdkdrd29pLWM/view?usp=sharing)<br/>
<br/>
Linux port hint:<br/>
[Ubuntu Kinect C++ Development Enviroment](https://github.com/suiwenfeng/Ubuntu_x64_Openni2.2_NiTE2.2_FreenectDriver).<br/>
<br/>
Windows port hint:<br/>
1.Download [Kinect for Windows SDK 1.8](https://msdn.microsoft.com/en-us/library/hh855347.aspx), install the SDK.<br/>
2.Download OpenNI 2 SDK Binaries from [https://structure.io/openni](https://structure.io/openni), install the SDK.<br/>
3.Download [NiTE-Windows-x64-2.2.zip](https://drive.google.com/file/d/0B3e4_6C5_YOjOGIySEluYkNibEE/edit) or [NiTE-Windows-x86-2.2.zip](https://drive.google.com/file/d/0B3e4_6C5_YOjQWtCcVl3VnRsWG8/edit), install the SDK.<br/>
4.Download [opencv 2.4.11](https://opencv.org/releases.html), install the SDK.<br/>
5.Build the source code with VS2010.<br/>
<br/>
Click the image to watch kinect mocap demo:<br/>
[![Kinect Mocap Demo](video-cover.png)](https://youtu.be/4x8NyXuXZWI "Kinect Mocap Demo")
<br/>
### Requirements
macOS 10.11<br/>
kinect for xbox 360<br/>
OpenNI2<br/>
NITE2<br/>
OpenCV2<br/>
Xcode 7.3.1
### How to use
1.Git clone [openni-nite-opencv-xcode](https://github.com/rishadbharucha/openni-nite-opencv-xcode).<br/>
2.Install libpng, OpenNI2, OpenCV2 from source code, via macport or homebrew.<br/>
3.Copy all the source codes to 'openni-nite-opencv-xcode/OpenNI+NITE+OpenCV', and add them to the project.<br/>
4.Copy /usr/local/lib/OpenNI2-FreenectDriver/libFreenectDriver.0.5.0.dylib to 'openni-nite-opencv-xcode/OpenNI2/Drivers/libFreenectDriver.dylib', then add the filename to Xcode's 'Targets->Build Phases->Copy Files(5 items)', ensure that the Subpath is 'OpenNI2/Drivers'.<br/>
5.Plug kinect for xbox 360 into USB port.<br/>
6.Open the project, build and run.<br/>
<br/>
Note: If you failed to run, just try again, the Kinect driver for macOS is not stable.<br/>
<br/>
When the Kinect sensor detects your body, the app will start recording, after the Kinect sensor lose your body, the app will stop recording, the BVH files will be saved to '../data'.<br/>
<br/>
Enter the Kinect camera's viewport, perform actions, then leave the viewport. Repeat the above steps, you can record many skeleton animation clips at one time.
### Thanks
1.[Derek Hendrickx's KinectMotionCapture](https://github.com/derekhendrickx/KinectMotionCapture)<br/>
2.[Kyle Weicht's 3D math library](https://github.com/awesomekyle/math)<br/>
3.[Birdy's Notebook](http://bediyap.com/programming/convert-quaternion-to-euler-rotations/)<br/>
4.[sunchy's Kinect_to_BVH_Console](https://github.com/isunchy/Kinect_to_BVH_Console)<br/>
5.[Rishad Bharucha's openni-nite-opencv-xcode](https://github.com/rishadbharucha/openni-nite-opencv-xcode)
### License
The MIT License (MIT)
