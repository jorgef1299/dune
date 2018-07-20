#ifndef MAPPING_H
#define MAPPING_H


using namespace std;

#include <Vision/FireMapper/Raster_Tile.h>
#include <Vision/FireMapper/Image.h>

struct Pixel_Test
{
  uchar Value;
  bool Test;
};


struct Corner_Test
{
  Pixel_Range PR;
  bool Test;
};

class Mapping
{

private:

  
  vector<Image> Images_to_Map;
  vector<cv::Mat> Images_Mapped;
  vector<Raster_Tile> Carte;
  vector < Raster_ALL > Liste;

public:
  Mapping();

  Pixel_Test Pixel_Mapping(Pixel_Data PD, int noDATA, Image IM);

  double IMask(cv::Mat UndistortedImage, std::vector<PixelImage> Corners);

  void Map(Image IM);

  vector<cv::Mat> get_IMapped();

  Point3D Raytracer(PixelImage Pix, Image I, Raster_Tile Rs);

  Corner_Test get_Imagecorners(Image IM, Raster_Tile RS);

  virtual ~Mapping();


};


#endif // MAPPING_H
