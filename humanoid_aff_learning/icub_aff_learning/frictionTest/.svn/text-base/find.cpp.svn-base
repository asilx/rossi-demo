#include <stdio.h>
#include <iostream>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

int
main ()
{
  Network yarp;
  BufferedPort<ImageOf<PixelRgb> > imagePort;
  BufferedPort<Vector> target;

  imagePort.open ("/target/image/in");
  target.open ("/target/out");
  Network::connect ("/icubSim/cam/left", "/target/image/in");

  // gets the position of an object from left eye of iCub, and sends this
  // to the port in "search.cpp"
  while (true)
  {
    ImageOf<PixelRgb> *image = imagePort.read ();

    if (image != NULL)
    {
      double xMean = 0;
      double yMean = 0;
      int ct = 0;

      for (int x = 0; x < image->width (); x++)
      {
        for (int y = 0; y < image->height (); y++)
        {
          PixelRgb& pixel = image->pixel (x, y);

          // selects the reddish object according to the threshold value
          if (pixel.r > pixel.b * 1.2 + 20 && pixel.r > pixel.g * 1.2 + 20)
          {
            xMean += x;
            yMean += y;
            ct++;
          }
        }
      }
      if (ct > 0)
      {
        xMean /= ct;
        yMean /= ct;
      }

      // to know whether an object is big enough to send its positions to the port
      if (ct > (image->width () / 35) * (image->height () / 35))
      {
        printf ("Best guess at red target: %g %g\n", xMean, yMean);
        Vector& tar = target.prepare ();
        tar.resize (3);
        tar[0] = xMean;
        tar[1] = yMean;
        tar[2] = 1;
        target.write ();
      }
      else
      {
        Vector& tar = target.prepare ();
        tar.resize (3);
        tar[0] = 0;
        tar[1] = 0;
        tar[2] = 0;
        target.write ();
      }
    }
    else
    {
      cout << "image is not detected!" << endl;
    }
  }

  return (0);
}
