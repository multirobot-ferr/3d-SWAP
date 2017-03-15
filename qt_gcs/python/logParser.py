##################################
##				##
##	UAV LOG PARSER		##
##				##
##################################

import sys
import numpy

## ------------------------------------------------------
def main():
	if(len(sys.argv) != 2):
		print("Please provide the file path as argument")
		return

	filePath = sys.argv[1]
	logFile = open(filePath, "r")

	altitudes = numpy.array([0.0])
	latitudes = numpy.array([0.0])
	longitudes = numpy.array([0.0])

	line = logFile.readline()
	while(line != ""):
		if "altitude" in line:
			if ":" in line:
				strs = line.split(':')
				strs[1] = strs[1].replace(',', '.')
				if(len(strs[1]) < 50):
					altitudes = numpy.append(altitudes, float(strs[1]))
		if "latitude" in line:
			if ":" in line:
				strs = line.split(':')
				strs[1] = strs[1].replace(',', '.')
				if(len(strs[1]) < 50):
					latitudes = numpy.append(latitudes, float(strs[1]))
		if "longitude" in line:
			if ":" in line:
				strs = line.split(':')
				strs[1] = strs[1].replace(',', '.')
				if(len(strs[1]) < 50):
					longitudes = numpy.append(longitudes, float(strs[1]))
	
		line = logFile.readline()

	data = numpy.row_stack((latitudes,longitudes))
	savePath = filePath.split('.')
	savePath = savePath[0] + "_processed.txt"
	numpy.savetxt(savePath,data , fmt="%f")


## ------------------------------------------------------
if __name__ == "__main__":
    main()
