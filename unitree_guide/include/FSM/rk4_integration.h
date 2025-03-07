#ifndef RK4_INTEGRATION_H
#define RK4_INTEGRATION_H

class RK4Integration {
public:
    RK4Integration(double initialX, double initialY, double stepSize);

    double integrateTo(double targetX);

    double getX() const;
    double getY() const;

private:
    double x;
    double y;
    double h;

    double f(double x, double y) const;
    double rk4(double x, double y, double h) const;
};

#endif // RK4_INTEGRATION_H
