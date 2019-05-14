# launch_xml

This package provides an abstraction of the XML tree.

## XML front-end mapping rules

### Accessing xml attributes

When having an xml tag like:

```xml
<tag attr="2"/>
```

If the entity `e` is wrapping it, the following statements will be true:
```python
e.get_attr('attr') == '2'
e.get_attr('attr', types='int') == 2
e.get_attr('attr', types='float') == 2.0
```

By default, the value of the attribute is returned as a string.
Allowed types are: 
```python
'str', 'int', 'float', 'bool', 'list[int]', 'list[float]', 'list[bool]', 'list[str]'
```
A combination of them can be specified with a tuple. e.g.: `('int', 'str')`.
In that case, conversions are tried in order and the first successful conversion is returned.
`types` can also be set to `guess`, which works in the same way as passing:

```python
'int', 'float', 'bool', 'list[int]', 'list[float]', 'list[bool]', 'list[str]', 'str'
```

For handling lists, the `*-sep` attribute is used. e.g.:

```xml
<tag attr="2,3,4" attr-sep=","/>
<tag2 attr="2 3 4" attr-sep=" "/>
<tag3 attr="2, 3, 4" attr-sep=", "/>
```

```python
tag.get_attr('attr', types='list[int]') == [2, 3, 4]
tag2.get_attr('attr', types='list[float]') == [2.0, 3.0, 4.0]
tag3.get_attr('attr', types='list[str]') == ['2', '3', '4']
```

For checking if an attribute exists, use optional argument:

```python
attr = e.get_attr('attr', optional=True)
if attr is not None:
    do_something(attr)
```

With `optional=False` (default), `AttributeError` is raised if it is not found.

### Accessing XML children as attributes:

In this xml:

```xml
<executable cmd="ls">
    <env name="a" value="100"/>
    <env name="b" value="stuff"/>
</node>
```

The `env` children could be accessed like:

```python
env = e.get_attr('env', types='list[Entity]')
len(env) == 2
env[0].get_attr('name') == 'a'
env[0].get_attr('value') == '100'
env[1].get_attr('name') == 'b'
env[1].get_attr('value') == 'stuff'
```

In these cases, `e.env` is a list of entities, that can be accessed in the same abstract way.

### Accessing all the XML children:

All the children can be directly accessed:

```python
e.children
```

It returns a list of launch_xml.Entity wrapping each of the xml children.

## Built-in substitutions

See [this](https://github.com/ros2/design/blob/d3a35d7ea201721892993e85e28a5a223cdaa001/articles/151_roslaunch_xml.md) document.
