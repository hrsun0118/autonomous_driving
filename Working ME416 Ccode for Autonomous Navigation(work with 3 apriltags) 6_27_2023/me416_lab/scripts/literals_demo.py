#!/usr/bin/env python
""" Small script demoing literals in Python """

print "Strings"
a = "I'm listening"
b = "He said OK!"
print a + " " + b

print "Lists"
l = [1, 2, 'three', 'last']

print "First: ", l[0]
print "Last: ", l[-1]
print "Everything but last: ", l[0:-1]
print "Array with only the first element: ", l[0:1]

print "Dictionaries"
d = {'first': 10, 'third': 3, 'second': 5}
print d['first'], d['second'], d['third']

#A whacky example showing that you can mix types and use numbers as keys
d = {'a': 10, 'test': 'x', 1: 'one'}
print "Value for the key 'test'", d['test']
print "value for the key 1", d[1]
