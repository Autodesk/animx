# What's left to do?

## Build & Deployment
* ~~Build/Test against older versions of Maya~~
 * ~~The lib should work for Maya 2014 and later~~
* ~~Add rules to install .lib/.mll & scripts in user-specified directory~~
* What's the versioning scheme for this library?
* ~~Move export directives for win32 out of code and into CMakefiles (see. FindMaya module)~~

## Code
* ~~Change all namespace names to be lower case~~
 * ~~Easier to type; more consistent with camel-case~~
* ~~Revisit ag.cpp/.h~~
 * ~~Use of statis sMachineTolerance is messy; could impact thread-safety~~
 * ~~Removing this could simplify code - avoid need for volatile, etc.~~
 * ~~change the Ag namespace to something more generic (e.g., Nurbs)~~

## Documentation
* Improve test scripts description. It's not very clear at the moment why you'd use one over the other.
* Improve code commenting
 * ag.h/.cpp are quite terse at the moment

## Other
* ~~Come up with a new name that doesn't refer to Maya~~
