# launch_xml

This package provides an abstraction of the XML tree.

## XML front-end mapping rules

### Accessing xml attributes

When having an xml tag like:

```xml
<tag attr='2'/>
```

If the entity `e` is wrapping it, the following two statements would be true:
```python
hasattr(e, 'attr') == True
e.attr == '2'
```

As a general rule, the value of the attribute is returned as an string.
Conversion to `float` or `int` should be explicitly done in the parser method.
For handling lists, see `Built-in Substitutions` section.

### Accessing XML children as parameters:

In this xml:

```xml
<executable cmd="ls">
    <env name="a" value="100"/>
    <env name="b" value="stuff"/>
</node>
```

The `env` children could be accessed like:

```python
len(e.env) == 2
e.env[0].name == 'a'
e.env[0].value == '100'
e.env[1].name == 'b'
e.env[1].value == 'stuff'
```

In these cases, `e.env` is a list of entities, which could be accessed in the same abstract way.

### Accessing all the XML children:

All the children can be directly accessed:

```python
e.children
```

It returns a list of launch_xml.Entity wrapping each of the xml children.

### Built-in substitutions

See [this](https://github.com/ros2/design/blob/d3a35d7ea201721892993e85e28a5a223cdaa001/articles/151_roslaunch_xml.md) document.

Additional substitution, for handling lists:

`$(list [sep=,] a,b,c,d)`
: Substituted by a python list, splited by the separator that follows `sep=`.
  Default separator is `,`.
  This substitution is evaluated instantaneously, and not in a lazy way.
