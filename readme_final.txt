Completeness:
    All final project objectives fulfilled. 
    Types of objects in the final composition:
        Rigid boxes, a cylinder, and spheres
        Two strings of spheres joined by hinged joints
        Jacks of boxes
        Cloth
        Volumetric deformable bodies
            The torus supplied to use
            My own volumetric deformable cone which was generated in Maya. The cone used is included in the cone* files in the main folder. First I generated stl files, then converted them for TETMESH using http://www.greentoken.de/onlineconv/
    The normal speed video and slow video are in the video folder. They are about 10 seconds yeach, there size being limited by the amount of ram on my laptop. Looks like my bullet implementation is leaky, and after about 10 second it consumes about 700MB of RAM, and that is all I had free, even if I close everything else. The resolution is 1024x768.
    
    You will find that as starter code I used code from a book on Bullet and open GL called "Learning Game Physics with
    Bullet Physics and OpenGL". I found the structure of the code hugely beneficial in implementing a neat and OOP oriented project in this space. I only used the code from one of the beginning chapters and added on to it from there. I'm including the starter code I used in the Chapter3.2_CreatingOurFirstPhysicsObject so that if necessary you can verify that my additions have been substantial and profound. 
    
Directions on how to run:
    This is going to be roughly my final project submission, where I also had to include the tiff file library source and my TETMESH objects. Hence the large size.
    The solution can be run from VS 2012 desktop express, on Debug Win32 settings.
    Additional library directories as well as include directories will need to be updated manually to match your machine, I did not use the system variable method.
    