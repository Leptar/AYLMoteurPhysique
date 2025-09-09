#ifndef PARTICULE_H
#define PARTICULE_H

class Particule {
private:
    float _x, _y, _z;

    float _vx, _vy, _vz;

    float _ax, _ay, _az;

    float _inverseMasse;

public:
    Particule(float px = 0.0f, float py = 0.0f, float pz = 0.0f,
              float masse = 1.0f)
        : _x(px), _y(py), _z(pz),
          _vx(0.0f), _vy(0.0f), _vz(0.0f),
          _ax(0.0f), _ay(0.0f), _az(0.0f)
    {
        setMasse(masse);
    }

    float getX() const { return _x; }
    float getY() const { return _y; }
    float getZ() const { return _z; }
    void setPosition(float px, float py, float pz) { _x = px; _y = py; _z = pz; }

    float getVx() const { return _vx; }
    float getVy() const { return _vy; }
    float getVz() const { return _vz; }
    void setVitesse(float vx, float vy, float vz) { _vx = vx; _vy = vy; _vz = vz; }

    float getAx() const { return _ax; }
    float getAy() const { return _ay; }
    float getAz() const { return _az; }
    void setAcceleration(float ax, float ay, float az) { _ax = ax; ay = ay; az = az; }

    void setMasse(float masse) {
        if (masse <= 0.0f) _inverseMasse = 0.0f;  
        else _inverseMasse = 1.0f / masse;
    }
    float getInverseMasse() const { return _inverseMasse; }
};

#endif
