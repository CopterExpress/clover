// Browserified https://www.npmjs.com/package/json-to-pretty-yaml module

(function(){function r(e,n,t){function o(i,f){if(!n[i]){if(!e[i]){var c="function"==typeof require&&require;if(!f&&c)return c(i,!0);if(u)return u(i,!0);var a=new Error("Cannot find module '"+i+"'");throw a.code="MODULE_NOT_FOUND",a}var p=n[i]={exports:{}};e[i][0].call(p.exports,function(r){var n=e[i][1][r];return o(n||r)},p,p.exports,r,e,n,t)}return n[i].exports}for(var u="function"==typeof require&&require,i=0;i<t.length;i++)o(t[i]);return o}return r})()({1:[function(require,module,exports){
(function() {
    "use strict";

    var typeOf = require('remedial').typeOf;
    var trimWhitespace = require('remove-trailing-spaces');

    function stringify(data) {
        var handlers, indentLevel = '';

        handlers = {
            "undefined": function() {
                // objects will not have `undefined` converted to `null`
                // as this may have unintended consequences
                // For arrays, however, this behavior seems appropriate
                return 'null';
            },
            "null": function() {
                return 'null';
            },
            "number": function(x) {
                return x;
            },
            "boolean": function(x) {
                return x ? 'true' : 'false';
            },
            "string": function(x) {
                // to avoid the string "true" being confused with the
                // the literal `true`, we always wrap strings in quotes
                return JSON.stringify(x);
            },
            "array": function(x) {
                var output = '';

                if (0 === x.length) {
                    output += '[]';
                    return output;
                }

                indentLevel = indentLevel.replace(/$/, '  ');
                x.forEach(function(y, i) {
                    // TODO how should `undefined` be handled?
                    var handler = handlers[typeOf(y)];

                    if (!handler) {
                        throw new Error('what the crap: ' + typeOf(y));
                    }

                    output += '\n' + indentLevel + '- ' + handler(y, true);

                });
                indentLevel = indentLevel.replace(/  /, '');

                return output;
            },
            "object": function(x, inArray, rootNode) {
                var output = '';

                if (0 === Object.keys(x).length) {
                    output += '{}';
                    return output;
                }

                if (!rootNode) {
                    indentLevel = indentLevel.replace(/$/, '  ');
                }

                Object.keys(x).forEach(function(k, i) {
                    var val = x[k],
                        handler = handlers[typeOf(val)];

                    if ('undefined' === typeof val) {
                        // the user should do
                        // delete obj.key
                        // and not
                        // obj.key = undefined
                        // but we'll error on the side of caution
                        return;
                    }

                    if (!handler) {
                        throw new Error('what the crap: ' + typeOf(val));
                    }

                    if (!(inArray && i === 0)) {
                        output += '\n' + indentLevel;
                    }

                    output += k + ': ' + handler(val);
                });
                indentLevel = indentLevel.replace(/  /, '');

                return output;
            },
            "function": function() {
                // TODO this should throw or otherwise be ignored
                return '[object Function]';
            }
        };
        return trimWhitespace(handlers[typeOf(data)](data, true, true) + '\n');

    }

    window.yamlStringify = stringify;
    module.exports.stringify = stringify;
}());

},{"remedial":2,"remove-trailing-spaces":3}],2:[function(require,module,exports){
/*jslint onevar: true, undef: true, nomen: true, eqeqeq: true, plusplus: true, bitwise: true, regexp: true, newcap: true, immed: true */
(function () {
    "use strict";

    var global = Function('return this')()
      , classes = "Boolean Number String Function Array Date RegExp Object".split(" ")
      , i
      , name
      , class2type = {}
      ;

    for (i in classes) {
      if (classes.hasOwnProperty(i)) {
        name = classes[i];
        class2type["[object " + name + "]"] = name.toLowerCase();
      }
    }

    function typeOf(obj) {
      return (null === obj || undefined === obj) ? String(obj) : class2type[Object.prototype.toString.call(obj)] || "object";
    }

    function isEmpty(o) {
        var i, v;
        if (typeOf(o) === 'object') {
            for (i in o) { // fails jslint
                v = o[i];
                if (v !== undefined && typeOf(v) !== 'function') {
                    return false;
                }
            }
        }
        return true;
    }

    if (!String.prototype.entityify) {
        String.prototype.entityify = function () {
            return this.replace(/&/g, "&amp;").replace(/</g,
                "&lt;").replace(/>/g, "&gt;");
        };
    }

    if (!String.prototype.quote) {
        String.prototype.quote = function () {
            var c, i, l = this.length, o = '"';
            for (i = 0; i < l; i += 1) {
                c = this.charAt(i);
                if (c >= ' ') {
                    if (c === '\\' || c === '"') {
                        o += '\\';
                    }
                    o += c;
                } else {
                    switch (c) {
                    case '\b':
                        o += '\\b';
                        break;
                    case '\f':
                        o += '\\f';
                        break;
                    case '\n':
                        o += '\\n';
                        break;
                    case '\r':
                        o += '\\r';
                        break;
                    case '\t':
                        o += '\\t';
                        break;
                    default:
                        c = c.charCodeAt();
                        o += '\\u00' + Math.floor(c / 16).toString(16) +
                            (c % 16).toString(16);
                    }
                }
            }
            return o + '"';
        };
    } 

    if (!String.prototype.supplant) {
        String.prototype.supplant = function (o) {
            return this.replace(/{([^{}]*)}/g,
                function (a, b) {
                    var r = o[b];
                    return typeof r === 'string' || typeof r === 'number' ? r : a;
                }
            );
        };
    }

    if (!String.prototype.trim) {
        String.prototype.trim = function () {
            return this.replace(/^\s*(\S*(?:\s+\S+)*)\s*$/, "$1");
        };
    }

    // CommonJS / npm / Ender.JS
    module.exports = {
        typeOf: typeOf,
        isEmpty: isEmpty
    };
    global.typeOf = global.typeOf || typeOf;
    global.isEmpty = global.isEmpty || isEmpty;
}());

},{}],3:[function(require,module,exports){
"use strict";

/**
 * removeTrailingSpaces
 * Remove the trailing spaces from a string.
 *
 * @name removeTrailingSpaces
 * @function
 * @param {String} input The input string.
 * @returns {String} The output string.
 */

module.exports = function removeTrailingSpaces(input) {
  // TODO If possible, use a regex
  return input.split("\n").map(function (x) {
    return x.trimRight();
  }).join("\n");
};
},{}]},{},[1]);
