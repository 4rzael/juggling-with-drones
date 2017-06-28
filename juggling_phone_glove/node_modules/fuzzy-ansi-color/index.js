var x256 = require('x256');
var isarray = require('isarray');
var lookup = require('colornames');

var modes = {
    bright: 1,
    dim: 2,
    underscore: 4,
    blink: 5,
    reverse: 7,
    hidden: 8
};

module.exports = function (parts) {
    if (!isarray(parts)) parts = String(parts).split(/\s+/);
    var strs = [], m;
    
    var mode = 'fg';
    
    for (var i = 0; i < parts.length; i++) {
        var p = parts[i];
        if (p === 'reset') {
            strs.push('\x1b[00m');
        }
        else if (p === 'fg') {
            mode = 'fg';
        }
        else if (p === 'bg') {
            mode = 'bg';
        }
        else if (isarray(p)) {
            strs.push(wrap(p));
        }
        else if (m = /^rgb\s*\((\S+)/.exec(p)) {
            var rgb = m[1].split(/\s*,\s*/);
            for (var i = 0; i < rgb.length; i++) {
                rgb[i] = Number(rgb[i]);
            }
            strs.push(wrap(rgb));
        }
        else if (/^#/.test(p)) {
            strs.push(wrap(parseHex(p)));
        }
        else if (modes[p]) {
            strs.push('\x1b[' + modes[p] + 'm');
        }
        else if (m = lookup(p)) {
            strs.push(wrap(parseHex(m)));
        }
    }
    return Buffer(strs.join(''));
    
    function wrap (s) { 
        var m = { fg: '38', bg: '48' }[mode];
        return '\x1b[' + m + ';' + 5 + ';' + x256(s) + 'm';
    }
};

function fromHex (s) {
    return parseInt(s, 16);
}

function parseHex (s) {
    var xs = s.replace(/^#/, '').match(/\w{2}/g);
    var res = [];
    for (var i = 0; i < xs.length; i++) res.push(fromHex(xs[i]));
    return res;
}
