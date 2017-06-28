var through = require('through2');
var colorize = require('../');

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
