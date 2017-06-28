var colorize = require('../');
process.stdout.write(Buffer.concat([
    colorize('bright cyan'), 
    Buffer('hello '),
    colorize('bg dim #c540a0 fg bright yellow'),
    Buffer('world\n'),
    colorize('reset')
]));
