#include "robot-config.h"
#include "math.h"
#include "algorithm"
vex::competition    Competition;

/*LIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARY
LIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLI
BRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBR
ARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRAR
LIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARYLIBRARY*/

int driveval;
int intakeval=0;
int capval=0;
int flipval=0;
int brakeval=0;

int tllshoot = -1;
int trlshoot = -1;
int bllshoot = -1;
int brlshoot = -1;

int tlrshoot = -1;
int trrshoot = -1;
int blrshoot = -1;
int brrshoot = -1;


int sfind()
{
    while(1)
    {
        Brain.Screen.printAt( 10, 20, "value: %d" , tllshoot);
        Brain.Screen.printAt( 10, 40, "value: %d" , trlshoot);
        Brain.Screen.printAt( 10, 60, "value: %d" , bllshoot);
        Brain.Screen.printAt( 10, 80, "value: %d" , brlshoot);
                             
        Brain.Screen.printAt( 10, 120, "value: %d" , tlrshoot);
        Brain.Screen.printAt( 10, 140, "value: %d" , trrshoot);
        Brain.Screen.printAt( 10, 160, "value: %d" , blrshoot);
        Brain.Screen.printAt( 10, 180, "value: %d" , brrshoot);

        Brain.Screen.printAt( 10, 220, "value: %d" , flipval);
    }
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

using namespace vex;
const double minimum_velocity = 20.0;

double mincreasing_speed (double starting_point, double current_position) 
{
    static const double macceleration_constant = 50.0;
    return macceleration_constant * std::abs(current_position - starting_point) + minimum_velocity;
}

double mdecreasing_speed (double ending_point, double current_position) 
{
    static const double mdeceleration_constant = 30.0;
    return mdeceleration_constant * std::abs(ending_point - current_position) + minimum_velocity;
}

double tincreasing_speed (double starting_point, double current_position) 
{
    static const double tacceleration_constant = 100.0;
    return tacceleration_constant * std::abs(current_position - starting_point) + minimum_velocity;
}

double tdecreasing_speed (double ending_point, double current_position) 
{
    static const double tdeceleration_constant = 50.0;
    return tdeceleration_constant * std::abs(ending_point - current_position) + minimum_velocity;
}

double pincreasing_speed (double starting_point, double current_position) 
{
    static const double acceleration_constant = 100.0;
    return acceleration_constant * std::abs(current_position - starting_point) + minimum_velocity;
}

double pdecreasing_speed (double ending_point, double current_position) 
{
    static const double deceleration_constant = 50.0;
    return deceleration_constant * std::abs(ending_point - current_position) + minimum_velocity;
}

double sincreasing_speed (double starting_point, double current_position) 
{
    static const double acceleration_constant = 100.0;
    return acceleration_constant * std::abs(current_position - starting_point) + minimum_velocity;
}

double sdecreasing_speed (double ending_point, double current_position) 
{
    static const double deceleration_constant = 100.0;
    return deceleration_constant * std::abs(ending_point - current_position) + minimum_velocity;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



void move (double distanceIn, double maxVelocity) {
    static const double circumference = 360;
    double direction = distanceIn > 0 ? 1.0 : -1.0;
    double wheelRevs = distanceIn / circumference;
    
    Rightdrivefront.spin(directionType::fwd,direction * minimum_velocity,velocityUnits::pct);
    Leftdrivefront.spin(directionType::fwd,direction * minimum_velocity,velocityUnits::pct);
    Rightdriveback.spin(directionType::fwd,direction * minimum_velocity,velocityUnits::pct);
    Leftdriveback.spin(directionType::fwd,direction * minimum_velocity,velocityUnits::pct);
    
    double leftStartPoint = Leftdrivefront.rotation(rotationUnits::rev);
    double leftEndPoint = leftStartPoint + wheelRevs;
    double rightStartPoint = Rightdrivefront.rotation(rotationUnits::rev);
    double rightEndPoint = rightStartPoint + wheelRevs;
    
    double leftBStartPoint = Leftdriveback.rotation(rotationUnits::rev);
    double leftBEndPoint = leftBStartPoint + wheelRevs;
    double rightBStartPoint = Rightdriveback.rotation(rotationUnits::rev);
    double rightBEndPoint = rightBStartPoint + wheelRevs;
    
    while (
            (direction * (Rightdrivefront.rotation(rotationUnits::rev) - rightStartPoint) < direction * wheelRevs) ||
            (direction * (Leftdrivefront.rotation(rotationUnits::rev) - leftStartPoint) < direction * wheelRevs)  ||
            (direction * (Leftdriveback.rotation(rotationUnits::rev) - leftBStartPoint) < direction * wheelRevs)  ||
            (direction * (Rightdriveback.rotation(rotationUnits::rev) - rightBStartPoint) < direction * wheelRevs)  
          ) 
    {
        if (direction * (Rightdrivefront.rotation(rotationUnits::rev) - rightStartPoint) < direction * wheelRevs) 
        {
            Rightdrivefront.setVelocity(
                direction * std::min(
                    maxVelocity,
                    std::min(
                        mincreasing_speed(rightStartPoint,Rightdrivefront.rotation(rotationUnits::rev)),
                        mdecreasing_speed(rightEndPoint,Rightdrivefront.rotation(rotationUnits::rev))
                    )
                ),
                vex::velocityUnits::pct
            );
        } 
        else 
        {
            Rightdrivefront.stop(brakeType::brake);
        }
        
        if (direction * (Leftdrivefront.rotation(rotationUnits::rev) - leftStartPoint) < direction * wheelRevs) 
        {
            Leftdrivefront.setVelocity(
                direction * std::min(
                    maxVelocity,
                    std::min(
                        mincreasing_speed(leftStartPoint,Leftdrivefront.rotation(rotationUnits::rev)),
                        mdecreasing_speed(leftEndPoint,Leftdrivefront.rotation(rotationUnits::rev))
                    )
                ),
                vex::velocityUnits::pct
            );
        } 
        else 
        {
            Leftdrivefront.stop(brakeType::brake);
        }
        
        if (direction * (Leftdriveback.rotation(rotationUnits::rev) - leftBStartPoint) < direction * wheelRevs) {
            Leftdriveback.setVelocity(
                direction * std::min(
                    maxVelocity,
                    std::min(
                        mincreasing_speed(leftBStartPoint,Leftdriveback.rotation(rotationUnits::rev)),
                        mdecreasing_speed(leftBEndPoint,Leftdriveback.rotation(rotationUnits::rev))
                    )
                ),
                vex::velocityUnits::pct
            );
        } 
        else 
        {
            Leftdriveback.stop(brakeType::brake);
        }
        
        if (direction * (Rightdriveback.rotation(rotationUnits::rev) - rightBStartPoint) < direction * wheelRevs) {
            Rightdriveback.setVelocity(
                direction * std::min(
                    maxVelocity,
                    std::min(
                        mincreasing_speed(rightBStartPoint,Rightdriveback.rotation(rotationUnits::rev)),
                        mdecreasing_speed(rightBEndPoint,Rightdriveback.rotation(rotationUnits::rev))
                    )
                ),
                vex::velocityUnits::pct
            );
        } 
        else 
        {
            Rightdriveback.stop(brakeType::brake);
        }
    }
    Leftdrivefront.stop(brakeType::brake);
    Leftdriveback.stop(brakeType::brake);
    Rightdrivefront.stop(brakeType::brake);
    Rightdriveback.stop(brakeType::brake);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void turn (double distanceIn, double maxVelocity) 
{
    static const double circumference = 360;
    double direction2 = distanceIn > 0 ? -1 : 1;
    double direction3 = distanceIn > 0 ? 1 : -1;
    double wheelRevs2 = (direction2*std::abs(distanceIn)) / circumference;
    double wheelRevs3 = (direction3*std::abs(distanceIn)) / circumference;
    
    Rightdrivefront.spin(directionType::fwd,direction2 * minimum_velocity,velocityUnits::pct);
    Leftdrivefront.spin(directionType::fwd,direction3 * minimum_velocity,velocityUnits::pct);
    Rightdriveback.spin(directionType::fwd,direction2* minimum_velocity,velocityUnits::pct);
    Leftdriveback.spin(directionType::fwd,direction3* minimum_velocity,velocityUnits::pct);
    
    double leftStartPoint = Leftdrivefront.rotation(rotationUnits::rev);
    double leftEndPoint = leftStartPoint + wheelRevs3;
    double rightStartPoint = Rightdrivefront.rotation(rotationUnits::rev);
    double rightEndPoint = rightStartPoint + wheelRevs2;
    
    double leftBStartPoint = Leftdriveback.rotation(rotationUnits::rev);
    double leftBEndPoint = leftBStartPoint + wheelRevs3;
    double rightBStartPoint = Rightdriveback.rotation(rotationUnits::rev);
    double rightBEndPoint = rightBStartPoint + wheelRevs2;
    
    while (
            (direction2* (Rightdrivefront.rotation(rotationUnits::rev) - rightStartPoint) < direction2* wheelRevs2) ||
            (direction3* (Leftdrivefront.rotation(rotationUnits::rev) - leftStartPoint) < direction3 * wheelRevs3)  ||
            (direction3* (Leftdriveback.rotation(rotationUnits::rev) - leftBStartPoint) < direction3 * wheelRevs3)  ||
            (direction2* (Rightdriveback.rotation(rotationUnits::rev) - rightBStartPoint) < direction2 * wheelRevs2)  
          ) 
    {
        if (direction2 * (Rightdrivefront.rotation(rotationUnits::rev) - rightStartPoint) < direction2 * wheelRevs2) 
        {
            Rightdrivefront.setVelocity(
                direction2 * std::min(
                    maxVelocity,
                    std::min(
                        tincreasing_speed(rightStartPoint,Rightdrivefront.rotation(rotationUnits::rev)),
                        tdecreasing_speed(rightEndPoint,Rightdrivefront.rotation(rotationUnits::rev))
                    )
                ),
                vex::velocityUnits::pct
            );
        } 
        else 
        {
            Rightdrivefront.stop(brakeType::brake);
        }
        
        if (direction3 * (Leftdrivefront.rotation(rotationUnits::rev) - leftStartPoint) < direction3 * wheelRevs3) 
        {
            Leftdrivefront.setVelocity(
                direction3 * std::min(
                    maxVelocity,
                    std::min(
                        tincreasing_speed(leftStartPoint,Leftdrivefront.rotation(rotationUnits::rev)),
                        tdecreasing_speed(leftEndPoint,Leftdrivefront.rotation(rotationUnits::rev))
                    )
                ),
                vex::velocityUnits::pct
            );
        } 
        else 
        {
            Leftdrivefront.stop(brakeType::brake);
        }
        
        if (direction3 * (Leftdriveback.rotation(rotationUnits::rev) - leftBStartPoint) < direction3 * wheelRevs3) {
            Leftdriveback.setVelocity(
                direction3 * std::min(
                    maxVelocity,
                    std::min(
                        tincreasing_speed(leftBStartPoint,Leftdriveback.rotation(rotationUnits::rev)),
                        tdecreasing_speed(leftBEndPoint,Leftdriveback.rotation(rotationUnits::rev))
                    )
                ),
                vex::velocityUnits::pct
            );
        } 
        else 
        {
            Leftdriveback.stop(brakeType::brake);
        }
        
        if (direction2 * (Rightdriveback.rotation(rotationUnits::rev) - rightBStartPoint) < direction2 * wheelRevs2) {
            Rightdriveback.setVelocity(
                direction2 * std::min(
                    maxVelocity,
                    std::min(
                        tincreasing_speed(rightBStartPoint,Rightdriveback.rotation(rotationUnits::rev)),
                        tdecreasing_speed(rightBEndPoint,Rightdriveback.rotation(rotationUnits::rev))
                    )
                ),
                vex::velocityUnits::pct
            );
        } 
        else 
        {
            Rightdriveback.stop(brakeType::brake);
        }
    }
    Leftdrivefront.stop(brakeType::brake);
    Leftdriveback.stop(brakeType::brake);
    Rightdrivefront.stop(brakeType::brake);
    Rightdriveback.stop(brakeType::brake);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void pointturn (double distanceIn, double maxVelocity, double sideval) 
{
    static const double circumference = 360;
    double direction4 = distanceIn > 0 ? 1.0 : -1.0;
    double wheelRevs4 = distanceIn / circumference;
    
    if (sideval == 1)
    {
        Leftdrivefront.stop(brakeType::hold);
        Leftdriveback.stop(brakeType::hold);
        Rightdrivefront.spin(directionType::fwd,direction4 * minimum_velocity,velocityUnits::pct);
        Rightdriveback.spin(directionType::fwd,direction4 * minimum_velocity,velocityUnits::pct);

        double rightStartPoint = Rightdrivefront.rotation(rotationUnits::rev);
        double rightEndPoint = rightStartPoint + wheelRevs4;
        double rightBStartPoint = Rightdriveback.rotation(rotationUnits::rev);
        double rightBEndPoint = rightBStartPoint + wheelRevs4;

        while (
                (direction4 * (Rightdrivefront.rotation(rotationUnits::rev) - rightStartPoint) < direction4 * wheelRevs4) ||
                (direction4 * (Rightdriveback.rotation(rotationUnits::rev) - rightBStartPoint) < direction4 * wheelRevs4)  
              ) 
        {
            if (direction4 * (Rightdrivefront.rotation(rotationUnits::rev) - rightStartPoint) < direction4 * wheelRevs4) 
            {
                Rightdrivefront.setVelocity(
                    direction4 * std::min(
                        maxVelocity,
                        std::min(
                            pincreasing_speed(rightStartPoint,Rightdrivefront.rotation(rotationUnits::rev)),
                            pdecreasing_speed(rightEndPoint,Rightdrivefront.rotation(rotationUnits::rev))
                        )
                    ),
                    vex::velocityUnits::pct
                );
            } 
            else 
            {
                Rightdrivefront.stop(brakeType::brake);
            }

            if (direction4 * (Rightdriveback.rotation(rotationUnits::rev) - rightBStartPoint) < direction4 * wheelRevs4) {
                Rightdriveback.setVelocity(
                    direction4 * std::min(
                        maxVelocity,
                        std::min(
                            pincreasing_speed(rightBStartPoint,Rightdriveback.rotation(rotationUnits::rev)),
                            pdecreasing_speed(rightBEndPoint,Rightdriveback.rotation(rotationUnits::rev))
                        )
                    ),
                    vex::velocityUnits::pct
                );
            } 
            else 
            {
                Rightdriveback.stop(brakeType::brake);
            }     
        }
        Rightdrivefront.stop(brakeType::brake);
        Rightdriveback.stop(brakeType::brake);
    }
    
    if (sideval == -1)
    {
        Rightdrivefront.stop(brakeType::hold);
        Rightdriveback.stop(brakeType::hold);
        Leftdrivefront.spin(directionType::fwd,direction4 * minimum_velocity,velocityUnits::pct);
        Leftdriveback.spin(directionType::fwd,direction4 * minimum_velocity,velocityUnits::pct);

        double leftStartPoint = Leftdrivefront.rotation(rotationUnits::rev);
        double leftEndPoint = leftStartPoint + wheelRevs4;

        double leftBStartPoint = Leftdriveback.rotation(rotationUnits::rev);
        double leftBEndPoint = leftBStartPoint + wheelRevs4;

        while (
                (direction4 * (Leftdrivefront.rotation(rotationUnits::rev) - leftStartPoint) < direction4 * wheelRevs4)  ||
                (direction4 * (Leftdriveback.rotation(rotationUnits::rev) - leftBStartPoint) < direction4 * wheelRevs4)
              ) 
        {
            if (direction4 * (Leftdrivefront.rotation(rotationUnits::rev) - leftStartPoint) < direction4 * wheelRevs4) 
            {
                Leftdrivefront.setVelocity(
                    direction4 * std::min(
                        maxVelocity,
                        std::min(
                            pincreasing_speed(leftStartPoint,Leftdrivefront.rotation(rotationUnits::rev)),
                            pdecreasing_speed(leftEndPoint,Leftdrivefront.rotation(rotationUnits::rev))
                        )
                    ),
                    vex::velocityUnits::pct
                );
            } 
            else 
            {
                Leftdrivefront.stop(brakeType::brake);
            }

            if (direction4 * (Leftdriveback.rotation(rotationUnits::rev) - leftBStartPoint) < direction4 * wheelRevs4) {
                Leftdriveback.setVelocity(
                    direction4 * std::min(
                        maxVelocity,
                        std::min(
                            pincreasing_speed(leftBStartPoint,Leftdriveback.rotation(rotationUnits::rev)),
                            pdecreasing_speed(leftBEndPoint,Leftdriveback.rotation(rotationUnits::rev))
                        )
                    ),
                    vex::velocityUnits::pct
                );
            } 
            else 
            {
                Leftdriveback.stop(brakeType::brake);
            }

        }
        Leftdrivefront.stop(brakeType::brake);
        Leftdriveback.stop(brakeType::brake);
    }
    Leftdrivefront.stop(brakeType::brake);
    Leftdriveback.stop(brakeType::brake);
    Rightdrivefront.stop(brakeType::brake);
    Rightdriveback.stop(brakeType::brake);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void strafe (double distanceIn, double maxVelocity) 
{
    static const double circumference = 360;
    double direction5 = distanceIn > 0 ? -1 : 1;
    double direction6 = distanceIn > 0 ? 1 : -1;
    double wheelRevs5 = (direction5*std::abs(distanceIn)) / circumference;
    double wheelRevs6 = (direction6*std::abs(distanceIn)) / circumference;
    
    Rightdrivefront.spin(directionType::fwd,direction5 * minimum_velocity,velocityUnits::pct);
    Leftdrivefront.spin(directionType::fwd,direction6 * minimum_velocity,velocityUnits::pct);
    Rightdriveback.spin(directionType::fwd,direction6* minimum_velocity,velocityUnits::pct);
    Leftdriveback.spin(directionType::fwd,direction5* minimum_velocity,velocityUnits::pct);
    
    double leftStartPoint = Leftdrivefront.rotation(rotationUnits::rev);
    double leftEndPoint = leftStartPoint + wheelRevs6;
    double rightStartPoint = Rightdrivefront.rotation(rotationUnits::rev);
    double rightEndPoint = rightStartPoint + wheelRevs5;
    
    double leftBStartPoint = Leftdriveback.rotation(rotationUnits::rev);
    double leftBEndPoint = leftBStartPoint + wheelRevs5;
    double rightBStartPoint = Rightdriveback.rotation(rotationUnits::rev);
    double rightBEndPoint = rightBStartPoint + wheelRevs6;
    
    while (
            (direction5* (Rightdrivefront.rotation(rotationUnits::rev) - rightStartPoint) < direction5* wheelRevs5) ||
            (direction6* (Leftdrivefront.rotation(rotationUnits::rev) - leftStartPoint) < direction6 * wheelRevs6)  ||
            (direction5* (Leftdriveback.rotation(rotationUnits::rev) - leftBStartPoint) < direction5 * wheelRevs5)  ||
            (direction6* (Rightdriveback.rotation(rotationUnits::rev) - rightBStartPoint) < direction6 * wheelRevs6)  
          ) 
    {
        if (direction5 * (Rightdrivefront.rotation(rotationUnits::rev) - rightStartPoint) < direction5 * wheelRevs5) 
        {
            Rightdrivefront.setVelocity(
                direction5 * std::min(
                    maxVelocity,
                    std::min(
                        sincreasing_speed(rightStartPoint,Rightdrivefront.rotation(rotationUnits::rev)),
                        sdecreasing_speed(rightEndPoint,Rightdrivefront.rotation(rotationUnits::rev))
                    )
                ),
                vex::velocityUnits::pct
            );
        } 
        else 
        {
            Rightdrivefront.stop(brakeType::brake);
        }
        
        if (direction6 * (Leftdrivefront.rotation(rotationUnits::rev) - leftStartPoint) < direction6 * wheelRevs6) 
        {
            Leftdrivefront.setVelocity(
                direction6 * std::min(
                    maxVelocity,
                    std::min(
                        sincreasing_speed(leftStartPoint,Leftdrivefront.rotation(rotationUnits::rev)),
                        sdecreasing_speed(leftEndPoint,Leftdrivefront.rotation(rotationUnits::rev))
                    )
                ),
                vex::velocityUnits::pct
            );
        } 
        else 
        {
            Leftdrivefront.stop(brakeType::brake);
        }
        
        if (direction5 * (Leftdriveback.rotation(rotationUnits::rev) - leftBStartPoint) < direction5 * wheelRevs5) {
            Leftdriveback.setVelocity(
                direction5 *0.80* std::min(
                    maxVelocity,
                    std::min(
                        sincreasing_speed(leftBStartPoint,Leftdriveback.rotation(rotationUnits::rev)),
                        sdecreasing_speed(leftBEndPoint,Leftdriveback.rotation(rotationUnits::rev))
                    )
                ),
                vex::velocityUnits::pct
            );
        } 
        else 
        {
            Leftdriveback.stop(brakeType::brake);
        }
        
        if (direction6 * (Rightdriveback.rotation(rotationUnits::rev) - rightBStartPoint) < direction6 * wheelRevs6) {
            Rightdriveback.setVelocity(
                direction6 *0.80* std::min(
                    maxVelocity,
                    std::min(
                        sincreasing_speed(rightBStartPoint,Rightdriveback.rotation(rotationUnits::rev)),
                        sdecreasing_speed(rightBEndPoint,Rightdriveback.rotation(rotationUnits::rev))
                    )
                ),
                vex::velocityUnits::pct
            );
        } 
        else 
        {
            Rightdriveback.stop(brakeType::brake);
        }
    }
    Leftdrivefront.stop(brakeType::brake);
    Leftdriveback.stop(brakeType::brake);
    Rightdrivefront.stop(brakeType::brake);
    Rightdriveback.stop(brakeType::brake);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int tllshots()
{
    if (Controller2.ButtonUp.pressing())
    {
        tllshoot = 10;
    }
    if (tllshoot == 10 && Controller2.ButtonRight.pressing())
    {
        tllshoot = 12;
    }
    if (tllshoot == 10 && Controller2.ButtonLeft.pressing())
    {
        tllshoot = 13;
    }
    if (tllshoot == 10 && Controller2.ButtonDown.pressing())
    {
        tllshoot = 14;
    }
    return(0);
}

int trlshots()
{
    if (Controller2.ButtonRight.pressing())
    {
        trlshoot = 20;
    }
    if (trlshoot == 20 && Controller2.ButtonUp.pressing())
    {
        trlshoot = 21;
    }
    if (trlshoot == 20 && Controller2.ButtonLeft.pressing())
    {
        trlshoot = 23;
    }
    if (trlshoot == 20 && Controller2.ButtonDown.pressing())
    {
        trlshoot = 24;
    }
    return(0);
}

int bllshots()
{
    if (Controller2.ButtonLeft.pressing())
    {
        bllshoot = 30;
    }
    if (bllshoot == 30 && Controller2.ButtonUp.pressing())
    {
        bllshoot = 31;
    }
    if (bllshoot == 30 && Controller2.ButtonRight.pressing())
    {
        bllshoot = 32;
    }
    if (bllshoot == 30 && Controller2.ButtonDown.pressing())
    {
        bllshoot = 34;
    }
    return(0);
}

int brlshots()
{
    if (Controller2.ButtonDown.pressing())
    {
        brlshoot = 40;
    }
    if (brlshoot == 40 && Controller2.ButtonUp.pressing())
    {
        brlshoot = 41;
    }
    if (brlshoot == 40 && Controller2.ButtonRight.pressing())
    {
        brlshoot = 42;
    }
    if (brlshoot == 40 && Controller2.ButtonLeft.pressing())
    {
        brlshoot = 43;
    }
    return(0);
}


int tlrshots()
{
    if (Controller2.ButtonX.pressing())
    {
        tlrshoot = 10;
    }
    if (tlrshoot == 10 && Controller2.ButtonA.pressing())
    {
        tlrshoot = 12;
    }
    if (tlrshoot == 10 && Controller2.ButtonY.pressing())
    {
        tlrshoot = 13;
    }
    if (tlrshoot == 10 && Controller2.ButtonB.pressing())
    {
        tlrshoot = 14;
    }
    return(0);
}

int trrshots()
{
    if (Controller2.ButtonA.pressing())
    {
        trrshoot = 20;
    }
    if (trrshoot == 20 && Controller2.ButtonX.pressing())
    {
        trrshoot = 21;
    }
    if (trrshoot == 20 && Controller2.ButtonY.pressing())
    {
        trrshoot = 23;
    }
    if (trrshoot == 20 && Controller2.ButtonB.pressing())
    {
        trrshoot = 24;
    }
    return(0);
}

int blrshots()
{
    if (Controller2.ButtonY.pressing())
    {
        blrshoot = 30;
    }
    if (blrshoot == 30 && Controller2.ButtonX.pressing())
    {
        blrshoot = 31;
    }
    if (blrshoot == 30 && Controller2.ButtonA.pressing())
    {
        blrshoot = 32;
    }
    if (blrshoot == 30 && Controller2.ButtonB.pressing())
    {
        blrshoot = 34;
    }
    return(0);
}

int brrshots()
{
    if (Controller2.ButtonB.pressing())
    {
        brrshoot = 40;
    }
    if (brrshoot == 40 && Controller2.ButtonX.pressing())
    {
        brrshoot = 41;
    }
    if (brrshoot == 40 && Controller2.ButtonA.pressing())
    {
        brrshoot = 42;
    }
    if (brrshoot == 40 && Controller2.ButtonY.pressing())
    {
        brrshoot = 43;
    }
    return(0);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int mecadrive()
{
    if (Controller.ButtonL1.pressing())
    {
        driveval = 1;
    }

    else if (Controller.ButtonR1.pressing())
    {
        driveval = -1;
    }
    else
    {
        driveval = 0;
    }
       
    Leftdrivefront.spin(vex::directionType::fwd,Controller.Axis3.value()-(driveval*127), vex::velocityUnits::pct);
    Leftdriveback.spin(vex::directionType::fwd,Controller.Axis3.value()+(driveval*127), vex::velocityUnits::pct);
    Rightdrivefront.spin(vex::directionType::fwd,Controller.Axis2.value()+(driveval*127), vex::velocityUnits::pct);
    Rightdriveback.spin(vex::directionType::fwd,Controller.Axis2.value()-(driveval*127), vex::velocityUnits::pct);

    return(0);
}

int drivebrake()
{
    if (Controller.ButtonX.pressing() && brakeval==0 && Brain.timer(vex::timeUnits::msec)>300)
    {
        Brain.resetTimer();
        brakeval=1;
        
        Leftdrivefront.stop(brakeType::hold);
        Leftdriveback.stop(brakeType::hold);
        Rightdrivefront.stop(brakeType::hold);
        Rightdriveback.stop(brakeType::hold);
    }

    if (Controller.ButtonX.pressing() && brakeval==1 && Brain.timer(vex::timeUnits::msec)>300)
    {
        Brain.resetTimer();
        brakeval=0;
    
        Leftdrivefront.stop(brakeType::coast);
        Leftdriveback.stop(brakeType::coast);
        Rightdrivefront.stop(brakeType::coast);
        Rightdriveback.stop(brakeType::coast);
    }
    return(0);
}

int capflip()
{
    if (Controller.ButtonR2.pressing() && capval==0 && Brain.timer(vex::timeUnits::msec)>300)
    {
        Brain.resetTimer();
        capval=1;
        Angle.rotateTo(1000,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
        Angle.stop(vex::brakeType::hold); 

    }

    if (Controller.ButtonR2.pressing() && capval==1 && Brain.timer(vex::timeUnits::msec)>300)
    {
        Brain.resetTimer();
        capval=0;
        Angle.rotateTo(0,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
        Angle.stop(vex::brakeType::hold); 

    }
    return(0);
}

int intake()
{

    if (Controller.ButtonL2.pressing() && intakeval==0 && Brain.timer(vex::timeUnits::msec)>500)
    {
        Brain.resetTimer();
        Intake.spin(vex::directionType::fwd,200,vex::velocityUnits::rpm);
        intakeval=1;
    }

    if (Controller.ButtonL2.pressing() && intakeval==1 && Brain.timer(vex::timeUnits::msec)>500)
    {
        Brain.resetTimer();
        Intake.spin(vex::directionType::fwd,-200,vex::velocityUnits::rpm);
        intakeval=2;
    }

    if (Controller.ButtonL2.pressing() && intakeval==2 && Brain.timer(vex::timeUnits::msec)>500)
    {
        Brain.resetTimer();
        Intake.spin(vex::directionType::fwd,0,vex::velocityUnits::rpm);
        intakeval=0;
    }
    return(0);
}

int intake2()
{
    if (Controller2.ButtonL2.pressing())
    {
        Intake.spin(vex::directionType::fwd,0,vex::velocityUnits::rpm);
        intakeval=0;
    }
    return(0);
}


int capremove()
{

    if (Controller.ButtonA.pressing() && flipval==0 && Brain.timer(vex::timeUnits::msec)>500)
    {
        Brain.resetTimer();
        flipval=1;
        Descorer.rotateTo(450,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
        Descorer.stop(vex::brakeType::hold);

    }

    if (Controller.ButtonA.pressing() && flipval==1 && Brain.timer(vex::timeUnits::msec)>500)
    {
        Brain.resetTimer();
        flipval=2;
        Descorer.rotateTo(800,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
        Descorer.stop(vex::brakeType::hold);

    }

    if (Controller.ButtonA.pressing() && flipval==2 && Brain.timer(vex::timeUnits::msec)>500)
    {
        Brain.resetTimer();
        flipval=0;
        Descorer.rotateTo(0,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
        Descorer.stop(vex::brakeType::hold);

    }
    return(0);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int doubleshootfar()
{
    Leftdrivefront.stop(brakeType::hold);
    Leftdriveback.stop(brakeType::hold);
    Rightdrivefront.stop(brakeType::hold);
    Rightdriveback.stop(brakeType::hold);
    
    Shooter.setRotation(0, vex::rotationUnits::deg);
    Angle.startRotateTo(150,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
    Shooter.rotateTo(360,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
    
    Angle.startRotateTo(275,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
    Shooter.rotateTo(713,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
    Angle.rotateTo(0,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
    Shooter.stop(vex::brakeType::coast);
    Angle.stop(vex::brakeType::hold);
       
    if (Shooter.rotation(vex::rotationUnits::deg) < 713)
    {
        while(Shooter.rotation(vex::rotationUnits::deg) < 713)
        {
            Shooter.spin(vex::directionType::fwd,30,vex::velocityUnits::rpm);
        }
    }

    else if (Shooter.rotation(vex::rotationUnits::deg) > 713)
    {
        while(Shooter.rotation(vex::rotationUnits::deg) > 713)
        {
            Shooter.spin(vex::directionType::fwd,-30,vex::velocityUnits::rpm);
        }
    }
    
    Shooter.setRotation(0, vex::rotationUnits::deg);
    Shooter.stop(vex::brakeType::hold); 
    
    Leftdrivefront.stop(brakeType::coast);
    Leftdriveback.stop(brakeType::coast);
    Rightdrivefront.stop(brakeType::coast);
    Rightdriveback.stop(brakeType::coast);
    return(0);
}

int doubleshootclose()
{
    Leftdrivefront.stop(brakeType::hold);
    Leftdriveback.stop(brakeType::hold);
    Rightdrivefront.stop(brakeType::hold);
    Rightdriveback.stop(brakeType::hold);
    
    Shooter.setRotation(0, vex::rotationUnits::deg);
    Angle.startRotateTo(0,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
    Shooter.rotateTo(360,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
    
    Angle.startRotateTo(275,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
    Shooter.rotateTo(713,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
    Angle.rotateTo(0,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
    Shooter.stop(vex::brakeType::coast);
    Angle.stop(vex::brakeType::hold);
       
    if (Shooter.rotation(vex::rotationUnits::deg) < 713)
    {
        while(Shooter.rotation(vex::rotationUnits::deg) < 713)
        {
            Shooter.spin(vex::directionType::fwd,30,vex::velocityUnits::rpm);
        }
    }

    else if (Shooter.rotation(vex::rotationUnits::deg) > 713)
    {
        while(Shooter.rotation(vex::rotationUnits::deg) > 713)
        {
            Shooter.spin(vex::directionType::fwd,-30,vex::velocityUnits::rpm);
        }
    }
    
    Shooter.setRotation(0, vex::rotationUnits::deg);
    Shooter.stop(vex::brakeType::hold); 
    
    Leftdrivefront.stop(brakeType::coast);
    Leftdriveback.stop(brakeType::coast);
    Rightdrivefront.stop(brakeType::coast);
    Rightdriveback.stop(brakeType::coast);
    return(0);
}

int highflagalign()
{
    Leftdrivefront.stop(brakeType::hold);
    Leftdriveback.stop(brakeType::hold);
    Rightdrivefront.stop(brakeType::hold);
    Rightdriveback.stop(brakeType::hold);
    
    Shooter.setRotation(0, vex::rotationUnits::deg);
    Angle.startRotateTo(0,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
    Shooter.rotateTo(357,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
    Shooter.stop(vex::brakeType::coast);
    Angle.stop(vex::brakeType::hold);
    Angle.rotateTo(0,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
    Angle.stop(vex::brakeType::hold); 
    
    if (Shooter.rotation(vex::rotationUnits::deg) < 357)
    {
        while(Shooter.rotation(vex::rotationUnits::deg) < 357)
        {
            Shooter.spin(vex::directionType::fwd,30,vex::velocityUnits::rpm);
        }
    }

    else if (Shooter.rotation(vex::rotationUnits::deg) > 357)
    {
        while(Shooter.rotation(vex::rotationUnits::deg) > 357)
        {
            Shooter.spin(vex::directionType::fwd,-30,vex::velocityUnits::rpm);
        }
    }
    
    Shooter.setRotation(0, vex::rotationUnits::deg);
    Shooter.stop(vex::brakeType::hold); 
    
    Leftdrivefront.stop(brakeType::coast);
    Leftdriveback.stop(brakeType::coast);
    Rightdrivefront.stop(brakeType::coast);
    Rightdriveback.stop(brakeType::coast);
    return(0);
}

int lowflagalign()
{
    Leftdrivefront.stop(brakeType::hold);
    Leftdriveback.stop(brakeType::hold);
    Rightdrivefront.stop(brakeType::hold);
    Rightdriveback.stop(brakeType::hold);
    
    Shooter.setRotation(0, vex::rotationUnits::deg);
    Angle.startRotateTo(275,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
    Shooter.rotateTo(355,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
    Shooter.stop(vex::brakeType::coast);
    Angle.stop(vex::brakeType::hold);
    Angle.rotateTo(0,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
    Angle.stop(vex::brakeType::hold); 
    
    if (Shooter.rotation(vex::rotationUnits::deg) < 355)
    {
        while(Shooter.rotation(vex::rotationUnits::deg) < 355)
        {
            Shooter.spin(vex::directionType::fwd,30,vex::velocityUnits::rpm);
        }
    }

    else if (Shooter.rotation(vex::rotationUnits::deg) > 355)
    {
        while(Shooter.rotation(vex::rotationUnits::deg) > 355)
        {
            Shooter.spin(vex::directionType::fwd,-30,vex::velocityUnits::rpm);
        }
    }
    
    Shooter.setRotation(0, vex::rotationUnits::deg);
    Shooter.stop(vex::brakeType::hold); 
    
    Leftdrivefront.stop(brakeType::coast);
    Leftdriveback.stop(brakeType::coast);
    Rightdrivefront.stop(brakeType::coast);
    Rightdriveback.stop(brakeType::coast);
    return(0);
}

int highflagfaralign()
{
    Leftdrivefront.stop(brakeType::hold);
    Leftdriveback.stop(brakeType::hold);
    Rightdrivefront.stop(brakeType::hold);
    Rightdriveback.stop(brakeType::hold);
    
    Shooter.setRotation(0, vex::rotationUnits::deg);
    Angle.startRotateTo(150,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
    Shooter.rotateTo(355,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
    Shooter.stop(vex::brakeType::coast);
    Angle.stop(vex::brakeType::hold);
    Angle.rotateTo(0,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
    Angle.stop(vex::brakeType::hold);
    
    if (Shooter.rotation(vex::rotationUnits::deg) < 355)
    {
        while(Shooter.rotation(vex::rotationUnits::deg) < 355)
        {
            Shooter.spin(vex::directionType::fwd,30,vex::velocityUnits::rpm);
        }
    }

    else if (Shooter.rotation(vex::rotationUnits::deg) > 355)
    {
        while(Shooter.rotation(vex::rotationUnits::deg) > 355)
        {
            Shooter.spin(vex::directionType::fwd,-30,vex::velocityUnits::rpm);
        }
    }
    
    Shooter.setRotation(0, vex::rotationUnits::deg);
    Shooter.stop(vex::brakeType::hold); 
    
    Leftdrivefront.stop(brakeType::coast);
    Leftdriveback.stop(brakeType::coast);
    Rightdrivefront.stop(brakeType::coast);
    Rightdriveback.stop(brakeType::coast);
    return(0);
}

int lowflagfaralign()
{
    Leftdrivefront.stop(brakeType::hold);
    Leftdriveback.stop(brakeType::hold);
    Rightdrivefront.stop(brakeType::hold);
    Rightdriveback.stop(brakeType::hold);

    Shooter.setRotation(0, vex::rotationUnits::deg);
    Angle.startRotateTo(275,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
    Shooter.rotateTo(355,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
    Shooter.stop(vex::brakeType::coast);
    Angle.stop(vex::brakeType::hold);
    Angle.rotateTo(0,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
    Angle.stop(vex::brakeType::hold); 
    
    if (Shooter.rotation(vex::rotationUnits::deg) < 355)
    {
        while(Shooter.rotation(vex::rotationUnits::deg) < 355)
        {
            Shooter.spin(vex::directionType::fwd,30,vex::velocityUnits::rpm);
        }
    }

    else if (Shooter.rotation(vex::rotationUnits::deg) > 355)
    {
        while(Shooter.rotation(vex::rotationUnits::deg) > 355)
        {
            Shooter.spin(vex::directionType::fwd,-30,vex::velocityUnits::rpm);
        }
    }
    
    Shooter.setRotation(0, vex::rotationUnits::deg);
    Shooter.stop(vex::brakeType::hold); 
    
    Leftdrivefront.stop(brakeType::coast);
    Leftdriveback.stop(brakeType::coast);
    Rightdrivefront.stop(brakeType::coast);
    Rightdriveback.stop(brakeType::coast);
    return(0);
}

/*PREAUTONPREAUTONPREAUTONPREAUTONPREAUTONPREAUTONPREAUTONPREAUTONPREAUTONPREAUTONPREAUTONPREAUTONPREAUTONPREAUTONPREAUTONPREAUT
ONPREAUTONPREAUTONPREAUTONPREAUTONPREAUTONPREAUTONPREAUTONPREAUTONPREAUTONPREAUTONPREAUTONPREAUTONPREAUTONPREAUTONPREAUTONPREAUT
ONPREAUTONPREAUTONPREAUTONPREAUTONPREAUTONPREAUTONPREAUTONPREAUTONPREAUTONPREAUTONPREAUTONPREAUTONPREAUTONPREAUTONPREAUTONPREAUT
ONPREAUTONPREAUTONPREAUTONPREAUTONPREAUTONPREAUTONPREAUTONPREAUTONPREAUTONPREAUTONPREAUTONPREAUTONPREAUTONPREAUTONPREAUTONPREAUT
ONPREAUTONPREAUTONPREAUTONPREAUTONPREAUTONPREAUTONPREAUTONPREAUTONPREAUTONPREAUTONPREAUTONPREAUTONPREAUTONPREAUTONPREAUTONPREA*/

void pre_auton(void) 
{

}

/*AUTONOMOUSAUTONOMOUSAUTONOMOUSAUTONOMOUSAUTONOMOUSAUTONOMOUSAUTONOMOUSAUTONOMOUSAUTONOMOUSAUTONOMOUSAUTONOMOUSAUTONOMOURAUTONO
AUTONOMOUSAUTONOMOUSAUTONOMOUSAUTONOMOUSAUTONOMOUSAUTONOMOUSAUTONOMOUSAUTONOMOUSAUTONOMOUSAUTONOMOUSAUTONOMOUSAUTONOMOURAUTONOMO
USAUTONOMOUSAUTONOMOUSAUTONOMOUSAUTONOMOUSAUTONOMOUSAUTONOMOUSAUTONOMOUSAUTONOMOUSAUTONOMOUSAUTONOMOUSAUTONOMOURAUTONOMOUSAUTONO
AUTONOMOUSAUTONOMOUSAUTONOMOUSAUTONOMOUSAUTONOMOUSAUTONOMOUSAUTONOMOUSAUTONOMOUSAUTONOMOUSAUTONOMOUSAUTONOMOUSAUTONOMOURAUTONOMO
USAUTONOMOUSAUTONOMOUSAUTONOMOUSAUTONOMOUSAUTONOMOUSAUTONOMOUSAUTONOMOUSAUTONOMOUSAUTONOMOUSAUTONOMOUSAUTONOMOUSAUTONOMOUSAUTO*/

void autonomous(void) 
{
    Brain.resetTimer();
    Angle.resetRotation();
    vex::task find(sfind);
    Angle.stop(vex::brakeType::hold);
    Intake.spin(vex::directionType::fwd,200,vex::velocityUnits::rpm);
    
    move(900,100);
    vex::task::sleep(100);

    move(-200,100);
    vex::task::sleep(100);

    Shooter.setRotation(0, vex::rotationUnits::deg);
    Angle.startRotateTo(140,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
    Shooter.startRotateTo(250,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
    turn(315,100);   
    vex::task::sleep(100);

    Shooter.rotateTo(360,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
    Angle.startRotateTo(250,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
    Shooter.rotateTo(720,vex::rotationUnits::deg,100,vex::velocityUnits::pct);

    Angle.stop(vex::brakeType::hold);
    Angle.rotateTo(0,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
    Angle.stop(vex::brakeType::hold);

    if (Shooter.rotation(vex::rotationUnits::deg) < 720)
    {
        while(Shooter.rotation(vex::rotationUnits::deg) < 720)
        {
            Shooter.spin(vex::directionType::fwd,30,vex::velocityUnits::rpm);
        }
    }

    else if (Shooter.rotation(vex::rotationUnits::deg) > 720)
    {
        while(Shooter.rotation(vex::rotationUnits::deg) > 720)
        {
            Shooter.spin(vex::directionType::fwd,-30,vex::velocityUnits::rpm);
        }
    }

    Shooter.setRotation(0, vex::rotationUnits::deg);
    Shooter.stop(vex::brakeType::hold);   

    Angle.startRotateTo(875,vex::rotationUnits::deg,100,vex::velocityUnits::pct);

    move(-300,100);

    vex::task::sleep(100);

    pointturn(-620,100,-1);

    vex::task::sleep(100);

    move(350,25);

    vex::task::sleep(100);

    move(-50,100);
    vex::task::sleep(100);

    Angle.rotateTo(1050,vex::rotationUnits::deg,100,vex::velocityUnits::pct);

    move(50,100);
    vex::task::sleep(100);

    Angle.stop(vex::brakeType::hold);

    Angle.rotateTo(10,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
    Angle.stop(vex::brakeType::hold);


    strafe(50,100);
    vex::task::sleep(100);
    Shooter.startRotateTo(250,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
    turn (251,100);
    vex::task::sleep(100);
    Shooter.rotateTo(360,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
    Angle.startRotateTo(180,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
    Shooter.rotateTo(720,vex::rotationUnits::deg,100,vex::velocityUnits::pct);

    Angle.stop(vex::brakeType::hold);
    Angle.rotateTo(0,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
    Angle.stop(vex::brakeType::hold);

    if (Shooter.rotation(vex::rotationUnits::deg) < 720)
    {
        while(Shooter.rotation(vex::rotationUnits::deg) < 720)
        {
            Shooter.spin(vex::directionType::fwd,30,vex::velocityUnits::rpm);
        }
    }

    else if (Shooter.rotation(vex::rotationUnits::deg) > 720)
    {
        while(Shooter.rotation(vex::rotationUnits::deg) > 720)
        {
            Shooter.spin(vex::directionType::fwd,-30,vex::velocityUnits::rpm);
        }
    }

    Shooter.setRotation(0, vex::rotationUnits::deg);
    Shooter.stop(vex::brakeType::hold);   

}

/*USERCONTROLUSERCONTROLUSERCONTROLUSERCONTROLUSERCONTROLUSERCONTROLUSERCONTROLUSERCONTROLUSERCONTROLUSERCONTROLUSERCONTROLUSERC
ONTROLUSERCONTROLUSERCONTROLUSERCONTROLUSERCONTROLUSERCONTROLUSERCONTROLUSERCONTROLUSERCONTROLUSERCONTROLUSERCONTROLUSERCONTROLU
SERCONTROLUSERCONTROLUSERCONTROLUSERCONTROLUSERCONTROLUSERCONTROLUSERCONTROLUSERCONTROLUSERCONTROLUSERCONTROLUSERCONTROLUSERCONT
ROLUSERCONTROLUSERCONTROLUSERCONTROLUSERCONTROLUSERCONTROLUSERCONTROLUSERCONTROLUSERCONTROLUSERCONTROLUSERCONTROLUSERCONTROLUSER
CONTROLUSERCONTROLUSERCONTROLUSERCONTROLUSERCONTROLUSERCONTROLUSERCONTROLUSERCONTROLUSERCONTROLUSERCONTROLUSERCONTROLUSERCONTR*/

void usercontrol(void) 
{
    vex::task find(sfind);
    while (1) 
    {  
            vex::task mecadrives(mecadrive);
            vex::task brake(drivebrake);
        
            vex::task intakes(intake);
            vex::task intakes2(intake2);
            vex::task capremoves(capremove);
        
            vex::task tllshot(tllshots);
            vex::task trlshot(trlshots);
            vex::task bllshot(bllshots);
            vex::task brlshot(brlshots);
        
            vex::task tlrshot(tlrshots);
            vex::task trrshot(trrshots);
            vex::task blrshot(blrshots);
            vex::task brrshot(brrshots);


            if (Controller.ButtonR2.pressing())
            {
                vex::task capflips(capflip);
            }
            if (Controller.ButtonB.pressing())
            {
                vex::task doubleshootsfar(doubleshootfar);
            }
            if (Controller.ButtonY.pressing())
            {
                vex::task doubleshootsclose(doubleshootclose);
            }
            if (Controller.ButtonUp.pressing())
            {
                vex::task high(highflagalign);
            }
            if (Controller.ButtonRight.pressing())
            {
                vex::task low(lowflagalign);
            }
            if (Controller.ButtonLeft.pressing())
            {
                vex::task highfar(highflagfaralign);
            }
            if (Controller.ButtonDown.pressing())
            {
                vex::task lowfar(lowflagfaralign);
            }

            if ((tllshoot == 12 || trlshoot == 21) && Controller2.ButtonL1.pressing()) // top left and top right
            {
                
                pointturn(-100,100,-1);

                Shooter.setRotation(0, vex::rotationUnits::deg);
                Angle.startRotateTo(160,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
                Shooter.rotateTo(360,vex::rotationUnits::deg,100,vex::velocityUnits::pct);

                Shooter.stop(vex::brakeType::coast);
                Angle.stop(vex::brakeType::hold);

                turn(120,100);

                Shooter.rotateTo(720,vex::rotationUnits::deg,100,vex::velocityUnits::pct);

                Angle.stop(vex::brakeType::hold);
                Angle.rotateTo(0,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
                Angle.stop(vex::brakeType::hold);
                
                if (Shooter.rotation(vex::rotationUnits::deg) < 720)
                {
                    while(Shooter.rotation(vex::rotationUnits::deg) < 720)
                    {
                        Shooter.spin(vex::directionType::fwd,30,vex::velocityUnits::rpm);
                    }
                }

                else if (Shooter.rotation(vex::rotationUnits::deg) > 720)
                {
                    while(Shooter.rotation(vex::rotationUnits::deg) > 720)
                    {
                        Shooter.spin(vex::directionType::fwd,-30,vex::velocityUnits::rpm);
                    }
                }

                Shooter.setRotation(0, vex::rotationUnits::deg);
                Shooter.stop(vex::brakeType::hold); 
                               
                tllshoot = -1;
                trlshoot = -1;
                bllshoot = -1;
                brlshoot = -1;
                
                Leftdrivefront.stop(brakeType::coast);
                Leftdriveback.stop(brakeType::coast);
                Rightdrivefront.stop(brakeType::coast);
                Rightdriveback.stop(brakeType::coast);
  
            }

            if ((tllshoot == 13 || bllshoot == 31) && Controller2.ButtonL1.pressing()) // top left and bottom left
            {
                pointturn(-100,100,-1);

                Shooter.setRotation(0, vex::rotationUnits::deg);
                Angle.startRotateTo(160,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
                Shooter.rotateTo(360,vex::rotationUnits::deg,100,vex::velocityUnits::pct);

                Shooter.stop(vex::brakeType::coast);
                Angle.stop(vex::brakeType::hold);

                Angle.startRotateTo(275,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
                Shooter.rotateTo(720,vex::rotationUnits::deg,100,vex::velocityUnits::pct);

                Angle.stop(vex::brakeType::hold);
                Angle.rotateTo(0,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
                Angle.stop(vex::brakeType::hold);
                
                if (Shooter.rotation(vex::rotationUnits::deg) < 720)
                {
                    while(Shooter.rotation(vex::rotationUnits::deg) < 720)
                    {
                        Shooter.spin(vex::directionType::fwd,30,vex::velocityUnits::rpm);
                    }
                }

                else if (Shooter.rotation(vex::rotationUnits::deg) > 720)
                {
                    while(Shooter.rotation(vex::rotationUnits::deg) > 720)
                    {
                        Shooter.spin(vex::directionType::fwd,-30,vex::velocityUnits::rpm);
                    }
                }

                Shooter.setRotation(0, vex::rotationUnits::deg);
                Shooter.stop(vex::brakeType::hold); 
                               
                tllshoot = -1;
                trlshoot = -1;
                bllshoot = -1;
                brlshoot = -1;
                
                Leftdrivefront.stop(brakeType::coast);
                Leftdriveback.stop(brakeType::coast);
                Rightdrivefront.stop(brakeType::coast);
                Rightdriveback.stop(brakeType::coast);
            }

            if ((tllshoot == 14 || brlshoot == 41) && Controller2.ButtonL1.pressing()) // top left and bottom right
            {
   
                pointturn(-100,100,-1);

                Shooter.setRotation(0, vex::rotationUnits::deg);
                Angle.startRotateTo(160,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
                Shooter.rotateTo(360,vex::rotationUnits::deg,100,vex::velocityUnits::pct);

                Shooter.stop(vex::brakeType::coast);
                Angle.stop(vex::brakeType::hold);

                turn(120,100);  
                
                Angle.startRotateTo(275,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
                Shooter.rotateTo(720,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
                Angle.stop(vex::brakeType::hold);
                Angle.rotateTo(0,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
                Angle.stop(vex::brakeType::hold);
                
                if (Shooter.rotation(vex::rotationUnits::deg) < 720)
                {
                    while(Shooter.rotation(vex::rotationUnits::deg) < 720)
                    {
                        Shooter.spin(vex::directionType::fwd,30,vex::velocityUnits::rpm);
                    }
                }

                else if (Shooter.rotation(vex::rotationUnits::deg) > 720)
                {
                    while(Shooter.rotation(vex::rotationUnits::deg) > 720)
                    {
                        Shooter.spin(vex::directionType::fwd,-30,vex::velocityUnits::rpm);
                    }
                }

                Shooter.setRotation(0, vex::rotationUnits::deg);
                Shooter.stop(vex::brakeType::hold); 
                
                tllshoot = -1;
                trlshoot = -1;
                bllshoot = -1;
                brlshoot = -1;
                
                Leftdrivefront.stop(brakeType::coast);
                Leftdriveback.stop(brakeType::coast);
                Rightdrivefront.stop(brakeType::coast);
                Rightdriveback.stop(brakeType::coast);
            }

            if ((trlshoot == 23 || bllshoot == 32) && Controller2.ButtonL1.pressing()) // top right and bottom left
            {
                
                pointturn(-100,100,-1);

                Shooter.setRotation(0, vex::rotationUnits::deg);
                Angle.startRotateTo(275,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
                Shooter.rotateTo(360,vex::rotationUnits::deg,100,vex::velocityUnits::pct);

                Shooter.stop(vex::brakeType::coast);
                Angle.stop(vex::brakeType::hold);

                turn(120,100);  

                Angle.startRotateTo(160,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
                Shooter.rotateTo(720,vex::rotationUnits::deg,100,vex::velocityUnits::pct);

                Angle.stop(vex::brakeType::hold);
                Angle.rotateTo(0,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
                Angle.stop(vex::brakeType::hold);
                
                if (Shooter.rotation(vex::rotationUnits::deg) < 720)
                {
                    while(Shooter.rotation(vex::rotationUnits::deg) < 720)
                    {
                        Shooter.spin(vex::directionType::fwd,30,vex::velocityUnits::rpm);
                    }
                }

                else if (Shooter.rotation(vex::rotationUnits::deg) > 720)
                {
                    while(Shooter.rotation(vex::rotationUnits::deg) > 720)
                    {
                        Shooter.spin(vex::directionType::fwd,-30,vex::velocityUnits::rpm);
                    }
                }

                Shooter.setRotation(0, vex::rotationUnits::deg);
                Shooter.stop(vex::brakeType::hold); 
                
                tllshoot = -1;
                trlshoot = -1;
                bllshoot = -1;
                brlshoot = -1;
                
                Leftdrivefront.stop(brakeType::coast);
                Leftdriveback.stop(brakeType::coast);
                Rightdrivefront.stop(brakeType::coast);
                Rightdriveback.stop(brakeType::coast);
            }

            if ((trlshoot == 24 || brlshoot == 42) && Controller2.ButtonL1.pressing()) // top right and bottom right
            {
                
                pointturn(-140,100,1);

                Shooter.setRotation(0, vex::rotationUnits::deg);
                Angle.startRotateTo(160,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
                Shooter.rotateTo(360,vex::rotationUnits::deg,100,vex::velocityUnits::pct);

                Shooter.stop(vex::brakeType::coast);
                Angle.stop(vex::brakeType::hold);

                Angle.startRotateTo(275,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
                Shooter.rotateTo(720,vex::rotationUnits::deg,100,vex::velocityUnits::pct);

                Angle.stop(vex::brakeType::hold);
                Angle.rotateTo(0,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
                Angle.stop(vex::brakeType::hold);
                
                if (Shooter.rotation(vex::rotationUnits::deg) < 720)
                {
                    while(Shooter.rotation(vex::rotationUnits::deg) < 720)
                    {
                        Shooter.spin(vex::directionType::fwd,30,vex::velocityUnits::rpm);
                    }
                }

                else if (Shooter.rotation(vex::rotationUnits::deg) > 720)
                {
                    while(Shooter.rotation(vex::rotationUnits::deg) > 720)
                    {
                        Shooter.spin(vex::directionType::fwd,-30,vex::velocityUnits::rpm);
                    }
                }

                Shooter.setRotation(0, vex::rotationUnits::deg);
                Shooter.stop(vex::brakeType::hold); 
                
                tllshoot = -1;
                trlshoot = -1;
                bllshoot = -1;
                brlshoot = -1;
                
                Leftdrivefront.stop(brakeType::coast);
                Leftdriveback.stop(brakeType::coast);
                Rightdrivefront.stop(brakeType::coast);
                Rightdriveback.stop(brakeType::coast);
            }


            if ((bllshoot == 34 || brlshoot == 43) && Controller2.ButtonL1.pressing()) // bottom left and bottom right
            {
                pointturn(-100,100,-1);

                Shooter.setRotation(0, vex::rotationUnits::deg);
                Angle.startRotateTo(275,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
                Shooter.rotateTo(360,vex::rotationUnits::deg,100,vex::velocityUnits::pct);

                Shooter.stop(vex::brakeType::coast);
                Angle.stop(vex::brakeType::hold);

                turn(120,100);  
                
                Shooter.rotateTo(720,vex::rotationUnits::deg,100,vex::velocityUnits::pct);

                Angle.stop(vex::brakeType::hold);
                Angle.rotateTo(0,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
                Angle.stop(vex::brakeType::hold);
                
                if (Shooter.rotation(vex::rotationUnits::deg) < 720)
                {
                    while(Shooter.rotation(vex::rotationUnits::deg) < 720)
                    {
                        Shooter.spin(vex::directionType::fwd,30,vex::velocityUnits::rpm);
                    }
                }

                else if (Shooter.rotation(vex::rotationUnits::deg) > 720)
                {
                    while(Shooter.rotation(vex::rotationUnits::deg) > 720)
                    {
                        Shooter.spin(vex::directionType::fwd,-30,vex::velocityUnits::rpm);
                    }
                }

                Shooter.setRotation(0, vex::rotationUnits::deg);
                Shooter.stop(vex::brakeType::hold); 
                
                tllshoot = -1;
                trlshoot = -1;
                bllshoot = -1;
                brlshoot = -1;
                
                Leftdrivefront.stop(brakeType::coast);
                Leftdriveback.stop(brakeType::coast);
                Rightdrivefront.stop(brakeType::coast);
                Rightdriveback.stop(brakeType::coast);
            }

        

            if (tllshoot == 10 && Controller2.ButtonL1.pressing()) // top left
            {
                pointturn(-100,100,-1);
                
                Shooter.setRotation(0, vex::rotationUnits::deg);
                Angle.startRotateTo(160,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
                Shooter.rotateTo(360,vex::rotationUnits::deg,100,vex::velocityUnits::pct);

                Angle.stop(vex::brakeType::hold);
                Angle.rotateTo(0,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
                Angle.stop(vex::brakeType::hold);
                
                if (Shooter.rotation(vex::rotationUnits::deg) < 360)
                {
                    while(Shooter.rotation(vex::rotationUnits::deg) < 360)
                    {
                        Shooter.spin(vex::directionType::fwd,30,vex::velocityUnits::rpm);
                    }
                }

                else if (Shooter.rotation(vex::rotationUnits::deg) > 360)
                {
                    while(Shooter.rotation(vex::rotationUnits::deg) > 360)
                    {
                        Shooter.spin(vex::directionType::fwd,-30,vex::velocityUnits::rpm);
                    }
                }

                Shooter.setRotation(0, vex::rotationUnits::deg);
                Shooter.stop(vex::brakeType::hold); 
                
                tllshoot = -1;
                trlshoot = -1;
                bllshoot = -1;
                brlshoot = -1;
                
                Leftdrivefront.stop(brakeType::coast);
                Leftdriveback.stop(brakeType::coast);
                Rightdrivefront.stop(brakeType::coast);
                Rightdriveback.stop(brakeType::coast);
            }


            if (trlshoot == 20  && Controller2.ButtonL1.pressing()) // top right
            {
                pointturn(-140,100,1);
                
                Shooter.setRotation(0, vex::rotationUnits::deg);
                Angle.startRotateTo(160,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
                Shooter.rotateTo(360,vex::rotationUnits::deg,100,vex::velocityUnits::pct);

                Angle.stop(vex::brakeType::hold);
                Angle.rotateTo(0,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
                Angle.stop(vex::brakeType::hold);
                
                if (Shooter.rotation(vex::rotationUnits::deg) < 360)
                {
                    while(Shooter.rotation(vex::rotationUnits::deg) < 360)
                    {
                        Shooter.spin(vex::directionType::fwd,30,vex::velocityUnits::rpm);
                    }
                }

                else if (Shooter.rotation(vex::rotationUnits::deg) > 360)
                {
                    while(Shooter.rotation(vex::rotationUnits::deg) > 360)
                    {
                        Shooter.spin(vex::directionType::fwd,-30,vex::velocityUnits::rpm);
                    }
                }

                Shooter.setRotation(0, vex::rotationUnits::deg);
                Shooter.stop(vex::brakeType::hold); 
                               
                tllshoot = -1;
                trlshoot = -1;
                bllshoot = -1;
                brlshoot = -1;
                
                Leftdrivefront.stop(brakeType::coast);
                Leftdriveback.stop(brakeType::coast);
                Rightdrivefront.stop(brakeType::coast);
                Rightdriveback.stop(brakeType::coast);
            }

            if (bllshoot == 30 && Controller2.ButtonL1.pressing()) // bottom left
            {
                pointturn(-100,100,-1);
                
                Shooter.setRotation(0, vex::rotationUnits::deg);
                Angle.startRotateTo(275,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
                Shooter.rotateTo(360,vex::rotationUnits::deg,100,vex::velocityUnits::pct);

                Angle.stop(vex::brakeType::hold);
                Angle.rotateTo(0,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
                Angle.stop(vex::brakeType::hold);
                
                if (Shooter.rotation(vex::rotationUnits::deg) < 360)
                {
                    while(Shooter.rotation(vex::rotationUnits::deg) < 360)
                    {
                        Shooter.spin(vex::directionType::fwd,30,vex::velocityUnits::rpm);
                    }
                }

                else if (Shooter.rotation(vex::rotationUnits::deg) > 360)
                {
                    while(Shooter.rotation(vex::rotationUnits::deg) > 360)
                    {
                        Shooter.spin(vex::directionType::fwd,-30,vex::velocityUnits::rpm);
                    }
                }

                Shooter.setRotation(0, vex::rotationUnits::deg);
                Shooter.stop(vex::brakeType::hold); 
                
                
                tllshoot = -1;
                trlshoot = -1;
                bllshoot = -1;
                brlshoot = -1;
                
                Leftdrivefront.stop(brakeType::coast);
                Leftdriveback.stop(brakeType::coast);
                Rightdrivefront.stop(brakeType::coast);
                Rightdriveback.stop(brakeType::coast);
            }

            if (brlshoot == 40 && Controller2.ButtonL1.pressing()) // bottom right
            {
                pointturn(-140,100,1);
                
                Shooter.setRotation(0, vex::rotationUnits::deg);
                Angle.startRotateTo(275,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
                Shooter.rotateTo(360,vex::rotationUnits::deg,100,vex::velocityUnits::pct);

                Angle.stop(vex::brakeType::hold);
                Angle.rotateTo(0,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
                Angle.stop(vex::brakeType::hold);
                
                if (Shooter.rotation(vex::rotationUnits::deg) < 360)
                {
                    while(Shooter.rotation(vex::rotationUnits::deg) < 360)
                    {
                        Shooter.spin(vex::directionType::fwd,30,vex::velocityUnits::rpm);
                    }
                }

                else if (Shooter.rotation(vex::rotationUnits::deg) > 360)
                {
                    while(Shooter.rotation(vex::rotationUnits::deg) > 360)
                    {
                        Shooter.spin(vex::directionType::fwd,-30,vex::velocityUnits::rpm);
                    }
                }

                Shooter.setRotation(0, vex::rotationUnits::deg);
                Shooter.stop(vex::brakeType::hold); 

                tllshoot = -1;
                trlshoot = -1;
                bllshoot = -1;
                brlshoot = -1;
               
                Leftdrivefront.stop(brakeType::coast);
                Leftdriveback.stop(brakeType::coast);
                Rightdrivefront.stop(brakeType::coast);
                Rightdriveback.stop(brakeType::coast);
            }
        
        
        
            if ((tlrshoot == 12 || trrshoot == 21) && Controller2.ButtonL1.pressing()) // top left and top right
            {
                
                pointturn(-155,100,1);

                Shooter.setRotation(0, vex::rotationUnits::deg);
                Angle.startRotateTo(160,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
                Shooter.rotateTo(360,vex::rotationUnits::deg,100,vex::velocityUnits::pct);

                Shooter.stop(vex::brakeType::coast);
                Angle.stop(vex::brakeType::hold);

                turn(-115,100);  
                

                Shooter.rotateTo(720,vex::rotationUnits::deg,100,vex::velocityUnits::pct);

                Angle.stop(vex::brakeType::hold);
                Angle.rotateTo(0,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
                Angle.stop(vex::brakeType::hold);
                
                if (Shooter.rotation(vex::rotationUnits::deg) < 720)
                {
                    while(Shooter.rotation(vex::rotationUnits::deg) < 720)
                    {
                        Shooter.spin(vex::directionType::fwd,30,vex::velocityUnits::rpm);
                    }
                }

                else if (Shooter.rotation(vex::rotationUnits::deg) > 720)
                {
                    while(Shooter.rotation(vex::rotationUnits::deg) > 720)
                    {
                        Shooter.spin(vex::directionType::fwd,-30,vex::velocityUnits::rpm);
                    }
                }

                Shooter.setRotation(0, vex::rotationUnits::deg);
                Shooter.stop(vex::brakeType::hold); 
                               
                tlrshoot = -1;
                trrshoot = -1;
                blrshoot = -1;
                brrshoot = -1;
                
                Leftdrivefront.stop(brakeType::coast);
                Leftdriveback.stop(brakeType::coast);
                Rightdrivefront.stop(brakeType::coast);
                Rightdriveback.stop(brakeType::coast);
  
            }

            if ((tlrshoot == 13 || blrshoot == 31) && Controller2.ButtonL1.pressing()) // top left and bottom left
            {
                pointturn(-70,100,-1);

                Shooter.setRotation(0, vex::rotationUnits::deg);
                Angle.startRotateTo(160,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
                Shooter.rotateTo(360,vex::rotationUnits::deg,100,vex::velocityUnits::pct);

                Shooter.stop(vex::brakeType::coast);
                Angle.stop(vex::brakeType::hold);

                Angle.startRotateTo(275,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
                Shooter.rotateTo(720,vex::rotationUnits::deg,100,vex::velocityUnits::pct);

                Angle.stop(vex::brakeType::hold);
                Angle.rotateTo(0,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
                Angle.stop(vex::brakeType::hold);
                
                if (Shooter.rotation(vex::rotationUnits::deg) < 720)
                {
                    while(Shooter.rotation(vex::rotationUnits::deg) < 720)
                    {
                        Shooter.spin(vex::directionType::fwd,30,vex::velocityUnits::rpm);
                    }
                }

                else if (Shooter.rotation(vex::rotationUnits::deg) > 720)
                {
                    while(Shooter.rotation(vex::rotationUnits::deg) > 720)
                    {
                        Shooter.spin(vex::directionType::fwd,-30,vex::velocityUnits::rpm);
                    }
                }

                Shooter.setRotation(0, vex::rotationUnits::deg);
                Shooter.stop(vex::brakeType::hold); 
                
                tlrshoot = -1;
                trrshoot = -1;
                blrshoot = -1;
                brrshoot = -1;
                
                Leftdrivefront.stop(brakeType::coast);
                Leftdriveback.stop(brakeType::coast);
                Rightdrivefront.stop(brakeType::coast);
                Rightdriveback.stop(brakeType::coast);
            }

            if ((tlrshoot == 14 || brrshoot == 41) && Controller2.ButtonL1.pressing()) // top left and bottom right
            {
   
                pointturn(-155,100,1);

                Shooter.setRotation(0, vex::rotationUnits::deg);
                Angle.startRotateTo(275,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
                Shooter.rotateTo(360,vex::rotationUnits::deg,100,vex::velocityUnits::pct);

                Shooter.stop(vex::brakeType::coast);
                Angle.stop(vex::brakeType::hold);

                turn(-115,100);  
                
                Angle.startRotateTo(160,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
                Shooter.rotateTo(720,vex::rotationUnits::deg,100,vex::velocityUnits::pct);

                Angle.stop(vex::brakeType::hold);
                Angle.rotateTo(0,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
                Angle.stop(vex::brakeType::hold);
                
                if (Shooter.rotation(vex::rotationUnits::deg) < 720)
                {
                    while(Shooter.rotation(vex::rotationUnits::deg) < 720)
                    {
                        Shooter.spin(vex::directionType::fwd,30,vex::velocityUnits::rpm);
                    }
                }

                else if (Shooter.rotation(vex::rotationUnits::deg) > 720)
                {
                    while(Shooter.rotation(vex::rotationUnits::deg) > 720)
                    {
                        Shooter.spin(vex::directionType::fwd,-30,vex::velocityUnits::rpm);
                    }
                }

                Shooter.setRotation(0, vex::rotationUnits::deg);
                Shooter.stop(vex::brakeType::hold); 
                
                tlrshoot = -1;
                trrshoot = -1;
                blrshoot = -1;
                brrshoot = -1;
                
                Leftdrivefront.stop(brakeType::coast);
                Leftdriveback.stop(brakeType::coast);
                Rightdrivefront.stop(brakeType::coast);
                Rightdriveback.stop(brakeType::coast);
            }

            if ((trrshoot == 23 || blrshoot == 32) && Controller2.ButtonL1.pressing()) // top right and bottom left
            {
                
                pointturn(-155,100,1);

                Shooter.setRotation(0, vex::rotationUnits::deg);
                Angle.startRotateTo(160,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
                Shooter.rotateTo(360,vex::rotationUnits::deg,100,vex::velocityUnits::pct);

                Shooter.stop(vex::brakeType::coast);
                Angle.stop(vex::brakeType::hold);

                turn(-115,100);  
                
                Angle.startRotateTo(275,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
                Shooter.rotateTo(720,vex::rotationUnits::deg,100,vex::velocityUnits::pct);

                Angle.stop(vex::brakeType::hold);
                Angle.rotateTo(0,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
                Angle.stop(vex::brakeType::hold);
                
                if (Shooter.rotation(vex::rotationUnits::deg) < 720)
                {
                    while(Shooter.rotation(vex::rotationUnits::deg) < 720)
                    {
                        Shooter.spin(vex::directionType::fwd,30,vex::velocityUnits::rpm);
                    }
                }

                else if (Shooter.rotation(vex::rotationUnits::deg) > 720)
                {
                    while(Shooter.rotation(vex::rotationUnits::deg) > 720)
                    {
                        Shooter.spin(vex::directionType::fwd,-30,vex::velocityUnits::rpm);
                    }
                }

                Shooter.setRotation(0, vex::rotationUnits::deg);
                Shooter.stop(vex::brakeType::hold); 
                
                tlrshoot = -1;
                trrshoot = -1;
                blrshoot = -1;
                brrshoot = -1;
                
                Leftdrivefront.stop(brakeType::coast);
                Leftdriveback.stop(brakeType::coast);
                Rightdrivefront.stop(brakeType::coast);
                Rightdriveback.stop(brakeType::coast);
            }

            if ((trrshoot == 24 || brrshoot == 42) && Controller2.ButtonL1.pressing()) // top right and bottom right
            {
                
                pointturn(-155,100,1);

                Shooter.setRotation(0, vex::rotationUnits::deg);
                Angle.startRotateTo(160,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
                Shooter.rotateTo(360,vex::rotationUnits::deg,100,vex::velocityUnits::pct);

                Shooter.stop(vex::brakeType::coast);
                Angle.stop(vex::brakeType::hold);

                Angle.startRotateTo(275,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
                Shooter.rotateTo(720,vex::rotationUnits::deg,100,vex::velocityUnits::pct);

                Angle.stop(vex::brakeType::hold);
                Angle.rotateTo(0,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
                Angle.stop(vex::brakeType::hold);
                
                if (Shooter.rotation(vex::rotationUnits::deg) < 720)
                {
                    while(Shooter.rotation(vex::rotationUnits::deg) < 720)
                    {
                        Shooter.spin(vex::directionType::fwd,30,vex::velocityUnits::rpm);
                    }
                }

                else if (Shooter.rotation(vex::rotationUnits::deg) > 720)
                {
                    while(Shooter.rotation(vex::rotationUnits::deg) > 720)
                    {
                        Shooter.spin(vex::directionType::fwd,-30,vex::velocityUnits::rpm);
                    }
                }

                Shooter.setRotation(0, vex::rotationUnits::deg);
                Shooter.stop(vex::brakeType::hold); 
                
                tlrshoot = -1;
                trrshoot = -1;
                blrshoot = -1;
                brrshoot = -1;
                
                Leftdrivefront.stop(brakeType::coast);
                Leftdriveback.stop(brakeType::coast);
                Rightdrivefront.stop(brakeType::coast);
                Rightdriveback.stop(brakeType::coast);
            }


            if ((blrshoot == 34 || brrshoot == 43) && Controller2.ButtonL1.pressing()) // bottom left and bottom right
            {
                pointturn(-155,100,1);

                Shooter.setRotation(0, vex::rotationUnits::deg);
                Angle.startRotateTo(275,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
                Shooter.rotateTo(360,vex::rotationUnits::deg,100,vex::velocityUnits::pct);

                Shooter.stop(vex::brakeType::coast);
                Angle.stop(vex::brakeType::hold);

                turn(-115,100);  
                

                Shooter.rotateTo(720,vex::rotationUnits::deg,100,vex::velocityUnits::pct);

                Angle.stop(vex::brakeType::hold);
                Angle.rotateTo(0,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
                Angle.stop(vex::brakeType::hold);
                
                if (Shooter.rotation(vex::rotationUnits::deg) < 720)
                {
                    while(Shooter.rotation(vex::rotationUnits::deg) < 720)
                    {
                        Shooter.spin(vex::directionType::fwd,30,vex::velocityUnits::rpm);
                    }
                }

                else if (Shooter.rotation(vex::rotationUnits::deg) > 720)
                {
                    while(Shooter.rotation(vex::rotationUnits::deg) > 720)
                    {
                        Shooter.spin(vex::directionType::fwd,-30,vex::velocityUnits::rpm);
                    }
                }

                Shooter.setRotation(0, vex::rotationUnits::deg);
                Shooter.stop(vex::brakeType::hold); 
                
                tlrshoot = -1;
                trrshoot = -1;
                blrshoot = -1;
                brrshoot = -1;
                
                Leftdrivefront.stop(brakeType::coast);
                Leftdriveback.stop(brakeType::coast);
                Rightdrivefront.stop(brakeType::coast);
                Rightdriveback.stop(brakeType::coast);
            }

        

            if (tlrshoot == 10 && Controller2.ButtonL1.pressing()) // top left
            {
                pointturn(-70,100,-1);
                
                Shooter.setRotation(0, vex::rotationUnits::deg);
                Angle.startRotateTo(160,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
                Shooter.rotateTo(360,vex::rotationUnits::deg,100,vex::velocityUnits::pct);

                Angle.stop(vex::brakeType::hold);
                Angle.rotateTo(0,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
                Angle.stop(vex::brakeType::hold);
                
                if (Shooter.rotation(vex::rotationUnits::deg) < 360)
                {
                    while(Shooter.rotation(vex::rotationUnits::deg) < 360)
                    {
                        Shooter.spin(vex::directionType::fwd,30,vex::velocityUnits::rpm);
                    }
                }

                else if (Shooter.rotation(vex::rotationUnits::deg) > 360)
                {
                    while(Shooter.rotation(vex::rotationUnits::deg) > 360)
                    {
                        Shooter.spin(vex::directionType::fwd,-30,vex::velocityUnits::rpm);
                    }
                }

                Shooter.setRotation(0, vex::rotationUnits::deg);
                Shooter.stop(vex::brakeType::hold); 
                
                tlrshoot = -1;
                trrshoot = -1;
                blrshoot = -1;
                brrshoot = -1;
                
                Leftdrivefront.stop(brakeType::coast);
                Leftdriveback.stop(brakeType::coast);
                Rightdrivefront.stop(brakeType::coast);
                Rightdriveback.stop(brakeType::coast);
            }


            if (trrshoot == 20  && Controller2.ButtonL1.pressing()) // top right
            {
                pointturn(-155,100,1);
                
                Shooter.setRotation(0, vex::rotationUnits::deg);
                Angle.startRotateTo(160,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
                Shooter.rotateTo(360,vex::rotationUnits::deg,100,vex::velocityUnits::pct);

                Angle.stop(vex::brakeType::hold);
                Angle.rotateTo(0,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
                Angle.stop(vex::brakeType::hold);
                
                if (Shooter.rotation(vex::rotationUnits::deg) < 360)
                {
                    while(Shooter.rotation(vex::rotationUnits::deg) < 360)
                    {
                        Shooter.spin(vex::directionType::fwd,30,vex::velocityUnits::rpm);
                    }
                }

                else if (Shooter.rotation(vex::rotationUnits::deg) > 360)
                {
                    while(Shooter.rotation(vex::rotationUnits::deg) > 360)
                    {
                        Shooter.spin(vex::directionType::fwd,-30,vex::velocityUnits::rpm);
                    }
                }

                Shooter.setRotation(0, vex::rotationUnits::deg);
                Shooter.stop(vex::brakeType::hold); 
                               
                tlrshoot = -1;
                trrshoot = -1;
                blrshoot = -1;
                brrshoot = -1;
                
                Leftdrivefront.stop(brakeType::coast);
                Leftdriveback.stop(brakeType::coast);
                Rightdrivefront.stop(brakeType::coast);
                Rightdriveback.stop(brakeType::coast);
            }

            if (blrshoot == 30 && Controller2.ButtonL1.pressing()) // bottom left
            {
                pointturn(-70,100,-1);
                
                Shooter.setRotation(0, vex::rotationUnits::deg);
                Angle.startRotateTo(275,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
                Shooter.rotateTo(360,vex::rotationUnits::deg,100,vex::velocityUnits::pct);

                Angle.stop(vex::brakeType::hold);
                Angle.rotateTo(0,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
                Angle.stop(vex::brakeType::hold);
                
                if (Shooter.rotation(vex::rotationUnits::deg) < 360)
                {
                    while(Shooter.rotation(vex::rotationUnits::deg) < 360)
                    {
                        Shooter.spin(vex::directionType::fwd,30,vex::velocityUnits::rpm);
                    }
                }

                else if (Shooter.rotation(vex::rotationUnits::deg) > 360)
                {
                    while(Shooter.rotation(vex::rotationUnits::deg) > 360)
                    {
                        Shooter.spin(vex::directionType::fwd,-30,vex::velocityUnits::rpm);
                    }
                }

                Shooter.setRotation(0, vex::rotationUnits::deg);
                Shooter.stop(vex::brakeType::hold); 
                
                tlrshoot = -1;
                trrshoot = -1;
                blrshoot = -1;
                brrshoot = -1;
                
                Leftdrivefront.stop(brakeType::coast);
                Leftdriveback.stop(brakeType::coast);
                Rightdrivefront.stop(brakeType::coast);
                Rightdriveback.stop(brakeType::coast);
            }

            if (brrshoot == 40 && Controller2.ButtonL1.pressing()) // bottom right
            {
                pointturn(-155,100,1);
                
                Shooter.setRotation(0, vex::rotationUnits::deg);
                Angle.startRotateTo(275,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
                Shooter.rotateTo(360,vex::rotationUnits::deg,100,vex::velocityUnits::pct);

                Angle.stop(vex::brakeType::hold);
                Angle.rotateTo(0,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
                Angle.stop(vex::brakeType::hold);
                
                if (Shooter.rotation(vex::rotationUnits::deg) < 360)
                {
                    while(Shooter.rotation(vex::rotationUnits::deg) < 360)
                    {
                        Shooter.spin(vex::directionType::fwd,30,vex::velocityUnits::rpm);
                    }
                }

                else if (Shooter.rotation(vex::rotationUnits::deg) > 360)
                {
                    while(Shooter.rotation(vex::rotationUnits::deg) > 360)
                    {
                        Shooter.spin(vex::directionType::fwd,-30,vex::velocityUnits::rpm);
                    }
                }

                Shooter.setRotation(0, vex::rotationUnits::deg);
                Shooter.stop(vex::brakeType::hold); 
                
                tlrshoot = -1;
                trrshoot = -1;
                blrshoot = -1;
                brrshoot = -1;
               
                Leftdrivefront.stop(brakeType::coast);
                Leftdriveback.stop(brakeType::coast);
                Rightdrivefront.stop(brakeType::coast);
                Rightdriveback.stop(brakeType::coast);
            }

            if (Controller2.ButtonR1.pressing()) 
            {
                
                tllshoot = -1;
                trlshoot = -1;
                bllshoot = -1;
                brlshoot = -1;
                
                tlrshoot = -1;
                trrshoot = -1;
                blrshoot = -1;
                brrshoot = -1;
            }



        
        
        
    }
}

int main() 
{
    pre_auton();
    Competition.autonomous( autonomous );
    Competition.drivercontrol( usercontrol );           
    while(1) 
    {
        vex::task::sleep(100);
    }     
}

