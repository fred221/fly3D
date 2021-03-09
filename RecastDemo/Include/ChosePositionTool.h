#ifndef CHOSEPOSITIONTOOL_H
#define CHOSEPOSITIONTOOL_H

#include "Sample.h"
class ChosePositionTool : public SampleTool
{
	Sample* m_sample;

public:
	ChosePositionTool();
	~ChosePositionTool();
	
	virtual int type() { return TOOL_CHOSE_POSITION; }
	virtual void init(Sample* sample);
	virtual void reset();
	virtual void handleMenu();
	virtual void handleClick(const float* s, const float* p, bool shift);
	virtual void handleToggle();
	virtual void handleStep();
	virtual void handleUpdate(const float dt);
	virtual void handleRender();
	virtual void handleRenderOverlay(double* proj, double* model, int* view);

private:
	void num_printf(char* s,int slen,float f);
	void drawPosition(const float* pos, float r, float h, float c, const unsigned int col);
};
extern float gBuildbeginPos[3];
extern bool  gChoosePos;
#endif // OFFMESHCONNECTIONTOOL_H
