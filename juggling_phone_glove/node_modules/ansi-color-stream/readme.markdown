# ansi-color-stream

colorize a stream with ansi colors

# example

Here we create 3 streams and interleave the colored chunks.

``` js
var through = require('through2');
var colorize = require('ansi-color-stream');

var a = through();
var b = through();
var c = through();

a.pipe(colorize('bg bright pink fg purple')).pipe(process.stdout);
b.pipe(colorize('purple')).pipe(process.stdout);
c.pipe(colorize('bg cyan fg blue')).pipe(process.stdout);

a.write('AAA');
b.write('BBB');
c.write('CCC\n');

b.write('bb');
c.write('ccc');
a.write('a\n');
```

# methods

``` js
var colorize = require('ansi-color-stream')
```

## var stream = colorize(color)

Return a transform stream that adds `color` to each of the chunks. `color`
is a
[fuzzy string name or array](https://npmjs.org/package/fuzzy-ansi-color)
that describes the color to use.

# install

With [npm](https://npmjs.org) do:

```
npm install ansi-color-stream
```

# license

MIT
