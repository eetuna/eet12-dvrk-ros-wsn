#!/usr/bin/python
import csv
# Open a file
fo = open("foo.txt", "rw+")
print "Name of the file: ", fo.name

# Assuming file has following 5 lines
# This is 1st line
# This is 2nd line
# This is 3rd line
# This is 4th line
# This is 5th line

line = fo.readline()
print "Read Line: %s" % (line)

line = fo.readline(5)
print "Read Line: %s" % (line)

lines=fo.readlines()
print lines[1][1]
#print lines[2][1]

# Close opend file
fo.close()



fo = open("foo.csv", "rw+")
print "Name of the file: ", fo.name


number_lines = sum(1 for line in open('foo.csv'))
print number_lines

data=fo.readlines()
fo.close()

for row in csv.reader(data):
	print row[2]


# Assuming file has following 5 lines
# This is 1st line
# This is 2nd line
# This is 3rd line
# This is 4th line
# This is 5th line

#line = fo.readline()
#print "Read Line: %s" % (line)

#line = fo.readline(5)
#print "Read Line: %s" % (line)

'''
lines=fo.readlines()

print len(lines)

print len(lines[0])
print lines[1][1]
#print lines[2][2]
#print lines[3][3]
#print lines[4][5]
# Close opend file
for line in lines:
	currentline = line.split(",")
	print 5
	print currentline


fo.close()
number_lines = sum(1 for line in open('foo.txt'))
print number_lines
#for n in range

nposes = len(lines)
print nposes
'''