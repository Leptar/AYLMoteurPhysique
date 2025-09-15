#ifndef PARTICULE_H
#define PARTICULE_H

class Particule {
private:
    double _px;
    double _py;
    double _pz;

    double _vx;
    double _vy;
    double _vz;

    double _ax;
    double _ay;
    double _az;

    double _inverseMasse;

public:
    Particule(double px = 0.0, double py = 0.0, double pz = 0.0,
              double vx = 0.0, double vy = 0.0, double vz = 0.0,
              double ax = 0.0, double ay = 0.0, double az = 0.0,
              double masse = 1.0);

    double getX() const;
    double getY() const;
    double getZ() const;

    double getVx() const;
    double getVy() const;
    double getVz() const;

    double getAx() const;
    double getAy() const;
    double getAz() const;

    double getInverseMasse() const;

    void setPosition(double px, double py, double pz);

    void setVitesse(double vx, double vy, double vz);

    void setAcceleration(double ax, double ay, double az);

    void setMasse(double masse);
};

#endif 
