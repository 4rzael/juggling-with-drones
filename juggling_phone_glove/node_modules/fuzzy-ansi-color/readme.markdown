# fuzzy-ansi-color

compute ansi color modes fuzzily with strings

# example

``` js
var colorize = require('fuzzy-ansi-color');
process.stdout.write(Buffer.concat([
    colorize('bright cyan'), 
    Buffer('hello '),
    colorize('bg dim #c540a0 fg bright yellow'),
    Buffer('world\n'),
    colorize('reset')
]));
```

# syntax

The syntax is a string that gets split on whitespace into an array of
commands.

Use a named color to set a color by its name. 

Use `/^#[0-9a-f]{6}/` to set the color with a hex code.

Use `rgb(\d+,\d+,\d+)` to set the color with an rgb code with values from 0
through 255, inclusive.

If you pass in an array for one of the parts, it will be treated as an rgb
code.

Set the mode with:

* `bright`
* `dim`
* `underscore`
* `blink`
* `reverse`
* `hidden`

Control whether to set the foreground or background mode with:

* `fg` 
* `bg`

Use the string `reset` to reset the color and mode.

# methods

``` js
var colorize = require('fuzzy-ansi-color')
```

## var buf = colorize(s)

Turn a string or array `s` into a colorized buffer `buf`.

# install

With [npm](https://npmjs.org/package/npm) do:

```
npm install fuzzy-ansi-color
```

# license

MIT
