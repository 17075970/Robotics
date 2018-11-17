#include <qapplication.h>
#include <iostream>

#include "../detail/opengl/SceneViewer.h"


#include <../ax18a/AX18A-Kinematics.h>
#include <tcp/AX18ARemoteInterface.h>



using namespace std;


void renderCylinderZ(double fromZ, double toZ, double width, double resolution=8){
    glPushMatrix();
    GLUquadric* obj = gluNewQuadric();
    glTranslated(0,0,fromZ);
    gluCylinder(obj, width, width, toZ-fromZ, resolution, resolution);
    gluDeleteQuadric(obj);
    glPopMatrix();
}

void renderArrow(double scale = 1.0){
    double length = scale*0.6;
    double width = scale*0.02;
    GLUquadric* obj = gluNewQuadric();
    gluCylinder(obj, width, width, length, 6, 6);
    glPushMatrix();
    glTranslated(0.0,0.0,length);
    gluCylinder(obj, width*2, 0.0, width*3, 6, 6);
    glPopMatrix();
    gluDeleteQuadric(obj);
}

void renderCoordinates(double scale = 1.0){
    glPushAttrib(GL_CURRENT_BIT);
    glColor3d(0,0,1);
    renderArrow(scale); // z axis

    glPushMatrix();
    glRotated(-90, 1.0, 0.0, 0.0);
    glColor3d(0,1,0);
    renderArrow(scale); // y axis
    glPopMatrix();

    glPushMatrix();
    glRotated(90, 0.0, 1.0, 0.0);
    glColor3d(1,0,0);
    renderArrow(scale); // x axis
    glPopMatrix();

    glPopAttrib();
}

void fillDHMatGL(double *mat, double alpha, double a, double theta, double d){
    Eigen::MatrixXd m = AX18AKinematics::getTransformMatrixFromDH(alpha, a, theta, d);
    for(uint i=0;i<4;i++){
        for(uint j=0;j<4;j++){
            mat[i+4*j] = m(i,j); // insert in transposed order for OpenGL
        }
    }
}

void renderBox(double lowX, double lowY, double lowZ, double highX, double highY, double highZ){
    glPushMatrix();

    glBegin(GL_POLYGON);
    glNormal3d(0,0,-1);
    glVertex3f( lowX,  lowY, lowZ);       // P1
    glVertex3f( lowX,  highY, lowZ);       // P2
    glVertex3f( highX, highY, lowZ);       // P3
    glVertex3f( highX, lowY, lowZ);       // P4
    glEnd();
    glBegin(GL_POLYGON);
    glNormal3d(0,0,1);
    glVertex3f( highX, lowY,  highZ );
    glVertex3f( highX, highY, highZ );
    glVertex3f( lowX,  highY, highZ );
    glVertex3f( lowX,  lowY,  highZ );
    glEnd();
    glBegin(GL_POLYGON);
    glNormal3d(1,0,0);
    glVertex3f( highX, lowY,  lowZ );
    glVertex3f( highX, highY, lowZ );
    glVertex3f( highX, highY, highZ );
    glVertex3f( highX, lowY,  highZ );
    glEnd();
    glBegin(GL_POLYGON);
    glNormal3d(-1,0,0);
    glVertex3f( lowX, lowY,  highZ );
    glVertex3f( lowX, highY, highZ );
    glVertex3f( lowX, highY, lowZ );
    glVertex3f( lowX, lowY,  lowZ );
    glEnd();
    glBegin(GL_POLYGON);
    glNormal3d(0,1,0);
    glVertex3f( highX, highY, highZ );
    glVertex3f( highX, highY, lowZ );
    glVertex3f( lowX,  highY, lowZ);
    glVertex3f( lowX,  highY, highZ );
    glEnd();
    glBegin(GL_POLYGON);
    glNormal3d(0,-1,0);
    glVertex3f( highX, lowY, lowZ );
    glVertex3f( highX, lowY, highZ );
    glVertex3f( lowX, lowY,  highZ );
    glVertex3f( lowX, lowY,  lowZ );
    glEnd();

    glPopMatrix();
}
void renderSphere(){
    glPushMatrix();
        GLUquadric* obj = gluNewQuadric();
        gluSphere(obj, 0.01, 6, 5);
        gluDeleteQuadric(obj);
    glPopMatrix();
}

class AX18Drawable : public Drawable {
public:
    AX18Drawable(std::shared_ptr<AX18ARobotInterface> robotP){
        obj = gluNewQuadric();
        robot = robotP;
    }
    ~AX18Drawable(){
        gluDeleteQuadric(obj);
    }
    void renderBase(){
        //base
        renderBox(-0.05, -0.05, -0.01, 0.05, 0.05, 0);
        //target1
        glPushMatrix();
        glTranslated(0, -0.2, 0.025);
        glColor3d(0.5, 0.5, 0.5);
        GLUquadric* obj = gluNewQuadric();
        gluSphere(obj, 0.008, 6, 5);
        glPopMatrix();
        gluDeleteQuadric(obj);
        //target2
        glPushMatrix();
        glTranslated(-0.2, 0, 0);
        glColor3d(0.5, 0.5, 0.5);
        obj = gluNewQuadric();
        gluSphere(obj, 0.008, 6, 5);
        glPopMatrix();
        gluDeleteQuadric(obj);
        //target3
        glPushMatrix();
        glTranslated(-0.2, 0, 0.2);
        glColor3d(0.5, 0.5, 0.5);
        obj = gluNewQuadric();
        gluSphere(obj, 0.008, 6, 5);
        glPopMatrix();
        gluDeleteQuadric(obj);
        //connection(to the second frame connection)
        double length = 0.015;
        double width = 0.005;
        glPushMatrix();
        glTranslated(0, 0, 0.025);
        glColor3d(0.5, 0.5, 0.5);
        obj = gluNewQuadric();
        gluCylinder(obj, width, width, length, 6, 6);
        glPopMatrix();
        gluDeleteQuadric(obj);
        //axis
        length = 0.02;
        width = 0.01;
        glPushMatrix();
        glColor3d(0.5, 0, 0);
        obj = gluNewQuadric();
        gluCylinder(obj, width, width, length, 6, 6);
        gluDeleteQuadric(obj);
        glPopMatrix();

    }
    void renderFrame1(){
        //connection
        double length = 0.055;
        double width = 0.005;
        glPushMatrix();
        glRotated(-90, 0, 1, 0);
        glRotated(45.5, 1, 0, 0);
        glTranslated(0, 0, 0.025);
        GLUquadric* obj = gluNewQuadric();
        gluCylinder(obj, width, width, length, 6, 6);
        gluDeleteQuadric(obj);
        glPopMatrix();
        //axis
        length = 0.02;
        width = 0.01;
        glPushMatrix();
        glColor3d(0.5, 0, 0);
        glTranslated(0, 0, -length / 2);
        obj = gluNewQuadric();
        gluCylinder(obj, width, width, length, 6, 6);
        gluDeleteQuadric(obj);
        glPopMatrix();


    }
    void renderFrame2(){
        //connection
        double length = 0.123;
        double width = 0.005;
        glPushMatrix();
        glRotated(-90, 0, 1, 0);
        glTranslated(0, 0, 0.025);
        glColor3f(0.5, 0.5, 0.5);
        GLUquadric* obj = gluNewQuadric();
        gluCylinder(obj, width, width, length, 6, 6);
        gluDeleteQuadric(obj);
        glPopMatrix();
        //axis
        length = 0.02;
        width = 0.01;
        glPushMatrix();
        glColor3d(0.5, 0, 0);
        glTranslated(0, 0, -length / 2);
        obj = gluNewQuadric();
        gluCylinder(obj, width, width, length, 6, 6);
        gluDeleteQuadric(obj);
        glPopMatrix();
    }
    void renderFrame3(){
        //axis
        double length = 0.02;
        double width = 0.01;
        glPushMatrix();
        glColor3d(0.5, 0, 0);
        glTranslated(0, 0, -length / 2);
        GLUquadric* obj = gluNewQuadric();
        gluCylinder(obj, width, width, length, 6, 6);
        gluDeleteQuadric(obj);
        glPopMatrix();
    }
    void renderFrame4(){
        //claw1
        double length = 0.05;
        double width = 0.003;
        glPushMatrix();
        glTranslated(0, -0.01, 0);
        glRotated(140, 1, 0, 0);
        GLUquadric* obj = gluNewQuadric();
        glColor3d(0.5, 0.5, 0.5);
        gluCylinder(obj, 0, width, length, 6, 6);
        glTranslated(0, 0, length);
        glColor3d(0.5, 0.5, 0.9);
        gluSphere(obj, 0.008, 6, 5);
        glRotated(80, 1, 0, 0);
        glColor3d(0.5, 0.5, 0.5);
        gluCylinder(obj, width, width, length, 6, 6);
        gluDeleteQuadric(obj);
        glPopMatrix();

        //claw2
        glPushMatrix();
        glRotated(180, 0, 0, 1);
        glTranslated(0, -0.01, 0);
        glRotated(140, 1, 0, 0);
        obj = gluNewQuadric();
        glColor3d(0.5, 0.5, 0.5);
        gluCylinder(obj, 0, width, length, 6, 6);
        glTranslated(0, 0, length);
        glColor3d(0.5, 0.5, 0.9);
        gluSphere(obj, 0.008, 6, 5);
        glRotated(80, 1, 0, 0);
        glColor3d(0.5, 0.5, 0.5);
        gluCylinder(obj, width, width, length, 6, 6);
        gluDeleteQuadric(obj);
        glPopMatrix();

        //connection
        glPushMatrix();
        glRotated(180, 0, 1, 0);
        glTranslated(0, 0, 0.07);
        glColor3d(0.5, 0.5, 0.5);
        gluCylinder(obj, 0.005, 0.005, 0.097, 6, 6);
        glPopMatrix();
    }
    virtual void draw(QGLViewer* viewer){

        Eigen::VectorXd q = robot->getCurrentJointAngles();

        glEnable(GL_DEPTH_TEST);

        renderCoordinates(0.1);
        robotMaterial();
        renderBase();

        for(int i=0; i<4; i++){
            glPushMatrix();
            double mat[16];
            fillDHMatGL(mat,
                        AX18AKinematics::alpha[i],
                        AX18AKinematics::a[i],
                        q[i]+AX18AKinematics::thetaOff[i],
                        AX18AKinematics::d[i]);
            glMultMatrixd(mat);

            renderCoordinates(0.1);

            robotMaterial();
            switch(i){
                case 0:
                    renderFrame1();
                    break;
                case 1:
                    renderFrame2();
                    break;
                case 2:
                    renderFrame3();
                    break;
                case 3:
                    renderFrame4();
                    break;
            }

        }
        for(int i=0; i<4; i++){
            glPopMatrix();
        }

    }
private:
    void robotMaterial(){
        glColor3d(0.5,0.5,0.5);

        float mat_ambient[] ={0.25f, 0.25f, 0.25f, 1.0f  };
        float mat_diffuse[] ={0.4f, 0.4f, 0.4f, 1.0f };
        float mat_specular[] ={0.774597f, 0.774597f, 0.774597f, 1.0f };
        float shine = 0.768f;
        glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
        glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);
        glMaterialf(GL_FRONT, GL_SHININESS, shine * 128.0);
    }

    double r,g,b;
    GLUquadric* obj;
    std::shared_ptr<AX18ARobotInterface> robot;
};

class SphereDrawable : public Drawable {
public:
    SphereDrawable(double x, double y, double z, int c){
        this->x = x;
        this->y = y;
        this->z = z;
        switch(c){
            case 1:
                r=1.0; g=0.0; b=0.0;
                break;
            case 2:
                r=0.0; g=1.0; b=0.0;
                break;
            case 3:
                r=0.0; g=0.0; b=1.0;
                break;
            default:
                r=0.5; g=0.5; b=0.5;
        }

        obj = gluNewQuadric();
    }
    ~SphereDrawable(){
        gluDeleteQuadric(obj);
    }
    virtual void draw(QGLViewer* viewer){

        glPushMatrix();
        glTranslated(x,y,z);

        //glShadeModel(GL_SMOOTH);
        //glShadeModel(GL_FLAT);

        const unsigned sphereSlices = 12;
        const unsigned sphereStacks = 12;
        const double radius = 0.05;
        glColor3d(r,g,b);
        gluSphere(obj, radius,sphereSlices,sphereStacks);


        glPopMatrix();
    }
private:
    double x;
    double y;
    double z;
    double r,g,b;
    GLUquadric* obj;
};

int main(int argc, char** argv){
    QApplication application(argc,argv);

    string remoteHost = "localhost";
    if(argc==2){
        remoteHost = argv[1];
    }

    boost::shared_ptr<SceneViewer> viewer = SceneViewer::create();

    //viewer->addDrawable(new SphereDrawable(0.0, 0.0, 0.0, 0));
    //viewer->addDrawable(new SphereDrawable(1.0, 0.0, 0.0, 1));
    //viewer->addDrawable(new SphereDrawable(0.0, 1.0, 0.0, 2));
    //viewer->addDrawable(new SphereDrawable(0.0, 0.0, 1.0, 3));

    viewer->addDrawable(new AX18Drawable(AX18ARemoteInterface::create(remoteHost)));

    viewer->setSceneRadius(5.0);
    viewer->show();
    viewer->restoreStateFromFile();

    return application.exec();
}
