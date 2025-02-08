// Basic GIS Bounding box class

class BoundingBox {
 public: 
  BoundingBox(double lonMin, double latMin, double lonMax, double latMax) : 
              m_lonMin(lonMin), m_lonMax(lonMax), m_latMin(latMin), m_latMax(latMax) {}

  bool contains(double lon, double lat) const {
    return lon >= this->m_lonMin && lat >= this->m_latMin && 
    lon <= this->m_lonMax && lat <= this->m_latMax;
  }

  double m_lonMin;
  double m_lonMax;
  double m_latMin;
  double m_latMax;
};

class Mapping {
 public:
  Mapping(BoundingBox bb, int column, int row): m_bb(bb), m_col(column), m_row(row) {
  
  }

  BoundingBox m_bb;
  int m_col;
  int m_row;

};