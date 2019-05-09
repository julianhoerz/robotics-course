

class Circle {
    public:
    double x;
    double y;
    float r;

    Circle(double x_coord, double y_coord, float radius){
        x = x_coord;
        y = y_coord;
        r = radius;
    }

    double getXCoord(){
        return x;
    }

    double getYCoord(){
        return y;
    }

    float getRadius(){
        return r;
    }
};