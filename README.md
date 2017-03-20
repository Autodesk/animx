# AnimX Animation Library

This library provides a set of function to calculate animation curve values in the exact same way as Maya. 
The library's main goal is to replicate interpolation schemes found in Maya. While an interface is provided to Maya's animation curves, curve storage, serialization, etc. is delegated to the client code. A python-based test suite is included to demonstrate how to use the library and ease validation against Maya.

# Building the Library

With the exception of cmake, which is used to ease building and deploying the library on different platforms, this library has no dependencies. The library does not require Maya and will build with many compilers. If you would like to use the included test suite (to validate the library generates the same answers as Maya), you should use a compiler that's compatible with the Maya version validating against. Please consult the Maya documentation for details and guidance about which compiler to use. 

Once you've downloaded cmake on your machine and have confirmed it's on the system path, you should be set. 

Note that you should specify a Maya version you have installed on your machine in order to have the test suite properly configured. In the below examples 2017 is used but valid values include also 2016, 2015 etc.

## OSX & Linux
Open a terminal and type:

```
mkdir build; cd build
cmake -DMAYA_VERSION=2017 ..
cmake --build . --config Release
```

## Windows
Open a command prompt and type:

```
mkdir build; cd build
cmake -G "Visual Studio 14 2015 Win64" -DMAYA_VERSION=2017 ..
```

Note: By default this library assumes you're using the same 32-bit scheme to encode time used by Maya, in version up to Maya 2017 update 3. If you plan to use the library with a version of Maya including or after Maya after 2017 update 3, define the symbol MAYA_64BIT_TIME_PRECISION when building the library. For example, linux and OSX users would issue the command:
```
cmake -DMAYA_VERSION=2017 -DMAYA_64BIT_TIME_PRECISION=1 ..
```
when following the steps described above.

Once built copy plugins and animation library DLL to the directory of your choice.

# Installation

After building the library, you can copy the generated binaries and python test scripts to a directory of your choosing by running:
```
cmake --install -DCMAKE_INSTALL_PREFIX=Path/to/destination ..
```
or for OSX & Linux as part of the build step (assuming CMAKE_INSTALL_PREFIX was already set):
```
cmake --build . --config Release --target install
```

Libraries will be placed directly at the destination folder while the scripts will go into a /scripts subfolder.

# Usage

To test the library inside Maya, two python scripts are included inside /tests/maya/scripts folder.

## Testing with Maya plugin

Load generated AnimXPlugin.mll plugin inside Maya and then source pyplugin.py script.
To launch an automatic test suite that will generate random curves and iterate over all tangents, interpolation and infinity modes, use

```python
testCurves()
```

## Testing with animation library

Instead of loading the plugin, you can also use pydll.py script. First specify the paths to both the animation library dll and the plugin mll in the script and then load it inside Maya. This script will load the dlls and call their interface functions directly.
To launch an automatic test suite, use

```python
testCurves()
```

Note: On Windows for this script to work, Maya's PATH variable must contain the path to the directory with compiled AnimX.dll. It can be set inside Maya.env file like so:
```
PATH = C:/Path/to/directory
```
