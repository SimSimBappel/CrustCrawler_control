function orthm = eulertoR(ang)

sa = sin(ang(1)); ca = cos(ang(1));
sb = sin(ang(2)); cb = cos(ang(2));
sc = sin(ang(3)); cc = cos(ang(3));

ra = [1  0   0;
      0  ca,  -sa; 
      0  sa, ca ];
		 
rb = [  cb,  0,  sb; ...
         0,  1,  0; ...
       -sb,  0,  cb];
	   
rc = [cc, -sc 0;
      sc, cc 0;
	    0 0 1];
orthm = rc*rb*ra; 