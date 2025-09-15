#include "Particule.h"

Particule::Particule(double px, double py, double pz,
                     double vx, double vy, double vz,
                     double ax, double ay, double az,
                     double masse)
    : _px(px), _py(py), _pz(pz),
      _vx(vx), _vy(vy), _vz(vz),
      _ax(ax), _ay(ay), _az(az) 
{
    setMasse(masse);
}

double Particule::getX() const { return _px; }
double Particule::getY() const { return _py; }
double Particule::getZ() const { return _pz; }

double Particule::getVx() const { return _vx; }
double Particule::getVy() const { return _vy; }
double Particule::getVz() const { return _vz; }

double Particule::getAx() const { return _ax; }
double Particule::getAy() const { return _ay; }
double Particule::getAz() const { return _az; }

double Particule::getInverseMasse() const { return _inverseMasse; }

void Particule::setPosition(double px, double py, double pz) {
    _px = px;
    _py = py;
    _pz = pz;
}

void Particule::setVitesse(double vx, double vy, double vz) {
    _vx = vx;
    _vy = vy;
    _vz = vz;
}

void Particule::setAcceleration(double ax, double ay, double az) {
    _ax = ax;
    _ay = ay;
    _az = az;
}

void Particule::setMasse(double masse) {
    if (masse <= 0.0) {
        _inverseMasse = 0.0;
    } else {
        _inverseMasse = 1.0 / masse;
    }
}
