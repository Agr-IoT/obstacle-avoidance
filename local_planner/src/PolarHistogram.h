
#include <memory>
#include <algorthm>


struct HSector
{
int sectorNum;
double minAngle;
double Hp;
};


struct BSector
{
int sectorNum;
bool occupied;
bool masked;
};


class HPHistogram
{

public:
    HPHistogram();
    ~HPHistogram():
    update();


private:
    float alpha;
    float numSectors;
    std::unique_ptr<HSector[]> pHistogram;


}