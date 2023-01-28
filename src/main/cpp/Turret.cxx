#include "Turret.hxx"

#include "Constants.hxx"
#include "Controllers.hxx"
#include "lib173/StateEstimator.hxx"

Turret::Turret()
{
    
}

Turret::~Turret(){
    
}

bool Turret::homingSwitchActive()
{
    return true;
}