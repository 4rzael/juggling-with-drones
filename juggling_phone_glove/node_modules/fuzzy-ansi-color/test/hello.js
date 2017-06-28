var test = require('tape');
var colorize = require('../');

test('hello', function (t) {
    t.plan(1);
    var buf = Buffer.concat([
        colorize('bright cyan'), 
        Buffer('hello '),
        colorize('bg dim #c540a0 fg bright yellow'),
        Buffer('world\n'),
        colorize('reset')
    ]);
    var expected = '\x1b[1m\x1b[38;5;51mhello \x1b[2m\x1b[48;5;169m'
        + '\x1b[1m\x1b[38;5;226mworld\n\x1b[00m'
    ;
    t.equal(buf.toString('utf8'), expected);
});
