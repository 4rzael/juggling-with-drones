var fuzzy = require('fuzzy-ansi-color');
var through = require('through2');

module.exports = function (rgb) {
    return through(function (buf, enc, next) {
        this.push(Buffer.concat([
            fuzzy(rgb),
            buf,
            fuzzy('reset')
        ]));
        next();
    });
};
