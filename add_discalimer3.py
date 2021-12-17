#!/usr/bin/python3
# -*- coding: utf-8 -*
import os
import sys
import re

regStr = '.*\*+\/'
claimer = None
claimer2= None
sufList = ['.c','.h','.cpp']
phyWord = b'Phyplus Microelectronics'
#claimerPath = os.path.join(os.getcwd(), 'Disclaimer.txt')
#claimerPath = os.path.join(os.getcwd(), 'empty_discalimer.txt')
#claimerPath2 = os.path.join(os.getcwd(), 'empty_discalimer_return.txt')
flist=[]
flist2=[]
flist3=[]

def findFile():
	for dirpath,dirnames,filenames in os.walk(os.getcwd()):
		for file in filenames:
			if os.path.splitext(file)[1] in sufList:
				fullpath = os.path.join(dirpath, file)
				addClaim(fullpath)


def addClaim(fullpath):
	content = ''
	pos1=-1
	pos2=-1
	#print(fullpath)
	with open(fullpath, "rb") as f: 
		content = f.read()
	
	# if phyWord in content :  
	# 	flist.append(fullpath)
	
	lenNR = 0
	if claimer in content :  
		flist.append(fullpath)
		pos1=content.find(claimer)
		#remove claimer
		lenClaimer = len(claimer)
		if(pos1>-1):
			while(1):
				if(content[pos1+1+lenClaimer+lenNR]==10 or content[pos1+1+lenClaimer+lenNR]==13):
					lenNR=lenNR+1
				else:
					break
			content = content[0:pos1]+content[pos1+lenNR+lenClaimer:len(content)]
	if claimer2 in content :  
		flist2.append(fullpath)
		if(pos1>-1):
			with open(fullpath, "wb") as f:
				f.write(content)
	else:
		if(pos1>-1):
			flist3.append(fullpath)
			content = claimer2 + '\n'.encode() + content
			with open(fullpath, "wb") as f:
				f.write(content)
	if(lenNR):
		print('remove return %d: %s'%(lenNR,fullpath))
		with open(fullpath, "wb") as f:
			f.write(content)
	#print(claimer in content)
#	if((claimer in content) is False):
#		content = claimer + '\n'.encode() + content
#		print('add discalimer: %s'%fullpath)
#		with open(fullpath, "wb") as f:
#			f.write(content)

def main(argv):
	global claimer
	global claimer2

	fname1 = (sys.argv[1])
	claimerPath = os.path.join(os.getcwd(), fname1)
	if(len(sys.argv)>2):
		fname2 = (sys.argv[2])
		claimerPath2 = os.path.join(os.getcwd(), fname2)
	else:
		fname2 = 'empty_header.txt'

	with open(claimerPath, "rb") as f: 
		claimer = f.read()
	with open(claimerPath2, "rb") as f: 
		claimer2 = f.read()
	
	findFile()

	print('File: %s'%claimerPath)
	for file in flist:
		print('->: %s'%file)
	print('Find Total %d '%(len(flist)))

	print('File: %s'%claimerPath2)
	for file in flist2:
		print('->: %s'%file)
	print('Find Total %d '%(len(flist2)))

	for file in flist3:
		print('<->: %s'%file)
	print('Replace Total %d '%(len(flist3)))

if __name__ == '__main__':
	main(sys.argv)