class Colour:

    #Constructors
    def __init__(self,name=""):
        self.name=name;
        self.h,self.s,self.v = 0,0,0;
        self.r,self.g,self.b = 0,0,0;

    #Getters
    def get_rgb(self):
        return self.r,self.g,self.b;

    def get_hsv(self):
        return self.h,self.s,self.v;

    #Setters
    def add_rgb(self,r,g,b):
        self.r = r;
        self.g = g;
        self.b = b;
        self.h,self.s,self.v = Colour.rgb_to_hsv(r,g,b);

    def add_hsv(self,h,s,v):
        self.h = h;
        self.s = s;
        self.v = v;
        self.r,self.g,self.b = Colour.hsv_to_rgb(h,s,v);

    def add_rgb_arr(self,colour):
        self.r = colour[0];
        self.g = colour[1];
        self.b = colour[2];
        self.h,self.s,self.v = Colour.rgb_to_hsv(colour[0],colour[1],colour[2]);

    def add_hsv_arr(self,colour):
        self.h = colour[0];
        self.s = colour[1];
        self.v = colour[2];
        self.r,self.g,self.b = Colour.hsv_to_rgb(colour[0],colour[1],colour[2]);

    #Operators
    def __eq__(self,colour):
        if colour.r==self.r and colour.g==self.g and colour.b==self.b:
            return True;
        return False;

    #Static functionality
    @staticmethod
    def hsv_to_rgb(h, s, v):
        if s == 0.0: v*=255; return (v, v, v)
        i = int(h*6.) # XXX assume int() truncates!
        f = (h*6.)-i; p,q,t = int(255*(v*(1.-s))), int(255*(v*(1.-s*f))), int(255*(v*(1.-s*(1.-f)))); v*=255; i%=6
        if i == 0: return (v, t, p)
        if i == 1: return (q, v, p)
        if i == 2: return (p, v, t)
        if i == 3: return (p, q, v)
        if i == 4: return (t, p, v)
        if i == 5: return (v, p, q)

    @staticmethod
    def rgb_to_hsv(r, g, b):
        r, g, b = r/255.0, g/255.0, b/255.0
        mx = max(r, g, b)
        mn = min(r, g, b)
        df = mx-mn
        if mx == mn:
            h = 0
        elif mx == r:
            h = (60 * ((g-b)/df) + 360) % 360
        elif mx == g:
            h = (60 * ((b-r)/df) + 120) % 360
        elif mx == b:
            h = (60 * ((r-g)/df) + 240) % 360
        if mx == 0:
            s = 0
        else:
            s = (df/mx)*100
        v = mx*100
        return h, s, v
