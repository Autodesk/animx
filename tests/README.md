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
