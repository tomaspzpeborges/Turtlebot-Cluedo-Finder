#!/usr/bin/env python

class Initialpoints:
	def __init__(self, path,filename):
		self.filename=filename;
		self.path = path;
		self.error=False;
		self.setup();

	def setup(self):
		self.f = open(self.path + self.filename,'r')
		lines = self.f.readlines();
		self.f.close();
		for line in lines:
			elems = line.split(":");
			if len(elems)==2:
				elems[0] = elems[0].replace(' ','');
				elems[1] = elems[1].replace(' ','');
				elems[1] = elems[1].replace('[','');
				elems[1] = elems[1].replace(']','');
				aux = elems[1].split(",");
				if elems[0]=="room1_entrance_xy":
					self.entrance1 = [float(aux[0]),float(aux[1])];
				elif elems[0]=="room2_entrance_xy":
					self.entrance2 = [float(aux[0]),float(aux[1])];
				elif elems[0]=="room1_centre_xy":
					self.center1 = [float(aux[0]),float(aux[1])];
				elif elems[0]=="room2_centre_xy":
					self.center2 = [float(aux[0]),float(aux[1])];
	
	def test_input(self):
		print(self.entrance1);
		print(self.entrance2);
		print(self.center1);
		print(self.center2);

#To be replaced by /map - OccupancyGrid msgs(from nav_msgs.msg import OccupancyGrid)
class PgmImage:

	def __init__(self, path,filename):
		self.filename=filename;
		self.path = path;
		self.error=False;
		self.setup();
		if self.error==False:
			self.read_image();

	def read_image(self):
		if self.type=="P5":
			self.read_binary();
		else:
			self.read_ascii();

	def read_binary(self):
		self.img = []
		for y in range(self.height):
			row = []
			for y in range(self.width):
				row.append(ord(self.f.read(1)))
			self.img.append(row)

	def read_ascii(self):
		# Read pixels information
		self.img = []
		buf = self.f.read()
		elem = buf.split()
		if len(elem) != self.width*self.height:
			print ('Error in number of pixels')
			self.error=True;
		for i in range(self.height):
			tmpList = []
			for j in range(self.width):
				tmpList.append(elem[i*self.width+j])
			self.img.append(tmpList)
		

	def setup(self):
		self.f = open(self.path + self.filename,'r')
		# Read header information
		count = 0
		while count < 3:
			line = self.f.readline()
			if line[0] == '#': # Ignore comments
				continue
			count = count + 1
			if count == 1: # Magic num info
				magicNum = line.strip()
			if magicNum != 'P2' and magicNum != 'P5':
				self.f.close()
				print ('Not a valid PGM file')
				self.error=True;
			elif count == 2: # Width and Height
				[width, height] = (line.strip()).split()
				self.width = int(width);
				self.height = int(height);
			elif count == 3: # Max gray level
				self.maxVal = int(line.strip())
		if self.error==False:
			self.type = magicNum;

	def display_colours(self):
		different =[];
		for i in self.img:
			for j in i:
				aux = False;
				for z in different:
					if z==j:
						aux=True;
				if aux==False:
					different.append(j);
		return different;

#To be replaced by /map_metadata - MapMetaData msgs(from nav_msgs.msg import MapMetaData)
class YamlFile:
	def __init__(self, path, filename):
		self.filename=filename;
		self.path = path;
		self.error=False;
		self.setup();
		self.img = PgmImage(self.path,filename[:-4]+"pgm");
		if self.error==False and self.img.error==False:
			self.generate_origin_points();
	
	def setup(self):
		self.f = open(self.path + self.filename,'r')
		lines = self.f.readlines();
		self.f.close();
		for line in lines:
			elems = line.split(":");
			if len(elems)==2:
				elems[0] = elems[0].replace(' ','');
				elems[1] = elems[1].replace(' ','');
				if elems[0]=="resolution":
					self.resolution = float(elems[1]);
				elif elems[0]=="origin":
					elems[1] = elems[1].replace('[','');
					elems[1] = elems[1].replace(']','');
					aux = elems[1].split(",");
					self.origin = [float(aux[0]),float(aux[1]),float(aux[2])];

	def generate_origin_points(self):
		self.x = self.origin[0] / self.resolution + self.img.width;
		self.y = self.origin[1] / self.resolution + self.img.height;
	
	def test_input(self):
		print(self.origin);
		print(self.resolution);
		print(self.x);
		print(self.y);
	





def pgmwrite(img, filename, maxVal=255, magicNum='P2'):
  """  This function writes a numpy array to a Portable GrayMap (PGM) 
  image file. By default, header number P2 and max gray level 255 are 
  written. Width and height are same as the size of the given list.
  Line1 : MagicNum
  Line2 : Width Height
  Line3 : Max Gray level
  Image Row 1
  Image Row 2 etc. """
  img = int32(img).tolist()
  f = open(filename,'w')
  width = 0
  height = 0
  for row in img:
    height = height + 1
    width = len(row)
  f.write(magicNum + '\n')
  f.write(str(width) + ' ' + str(height) + '\n')
  f.write(str(maxVal) + '\n')
  for i in range(height):
    count = 1
    for j in range(width):
      f.write(str(img[i][j]) + ' ')
      if count >= 17:
        # No line should contain gt 70 chars (17*4=68)
        # Max three chars for pixel plus one space
        count = 1
        f.write('\n')
      else:
        count = count + 1
    f.write('\n')
  f.close()




fileyaml = YamlFile("/world/map\\", "project_map.yaml")



print(fileyaml.img.display_colours());
print(fileyaml.test_input());
				