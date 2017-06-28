var test = require('tape');
var through = require('through2');
var colorize = require('../');

test('colorize', function (t) {
    t.plan(1);
    
    var a = through();
    var b = through();
    var c = through();
    var output = through(write);
    var chunks = [];

    a.pipe(colorize('bg bright pink fg purple')).pipe(output);
    b.pipe(colorize('purple')).pipe(output);
    c.pipe(colorize('bg cyan fg blue')).pipe(output);

    a.write('AAA');
    b.write('BBB');
    c.write('CCC\n');

    b.write('bb');
    c.write('ccc');
    a.write('a\n');
    
    function write (buf, enc, next) {
        chunks.push(buf);
        next();
    }
    
    t.equal(
        Buffer.concat(chunks).toString(),
        '\x1b[1m\x1b[48;5;218m\x1b[38;5;5mAAA\x1b[00m\x1b[38;5;5mBBB'
        + '\x1b[00m\x1b[48;5;51m\x1b[38;5;21mCCC\n'
        + '\x1b[00m\x1b[38;5;5mbb\x1b[00m\x1b[48;5;51m\x1b[38;5;21mccc'
        + '\x1b[00m\x1b[1m\x1b[48;5;218m\x1b[38;5;5ma\n\x1b[00m'
    );
});
