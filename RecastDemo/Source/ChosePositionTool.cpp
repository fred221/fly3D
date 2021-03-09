#include "ChosePositionTool.h"
#include "imgui.h"
float gBuildbeginPos[3] = {0.0};
bool gChoosePos = false;
ChosePositionTool::ChosePositionTool()
{

}

ChosePositionTool::~ChosePositionTool()
{

}

void ChosePositionTool::init(Sample* sample)
{
	m_sample = sample;
}

void ChosePositionTool::reset()
{}

void ChosePositionTool::handleMenu()
{
	imguiLabel("Begin position");

	char* format = "x = %s , y = %s, z = %s";
	char x_str[128],y_str[128],z_str[128];
	
	num_printf(x_str, 128, gBuildbeginPos[0]);
	num_printf(y_str, 128, gBuildbeginPos[1]);
	num_printf(z_str, 128, gBuildbeginPos[2]);

	char str[1024];
	sprintf(str, format, x_str, y_str, z_str);
	imguiLabel(str);
}
void ChosePositionTool::num_printf(char* s, int slen,float f)
{
	//static const float EPS = 0.0001f;
	//int digits = 2;
	//char fmt[16];
	//snprintf(fmt, 16, "%%.%df", digits >= 0 ? 0 : -digits);
	snprintf(s, slen, "%.2f", f);
}
void ChosePositionTool::handleClick(const float* s, const float* p, bool shift)
{
	rcVcopy(gBuildbeginPos, p);
	gChoosePos = true;
}

void ChosePositionTool::handleToggle() {}

void ChosePositionTool::handleStep()
{}

void ChosePositionTool::handleUpdate(const float dt)
{}

void ChosePositionTool::handleRender()
{
	duDebugDraw& dd = m_sample->getDebugDraw();

	static const unsigned int startCol = duRGBA(128, 25, 0, 192);
	static const unsigned int endCol = duRGBA(51, 102, 0, 129);
	static const unsigned int pathCol = duRGBA(0, 0, 0, 64);

	const float agentRadius = m_sample->getAgentRadius();
	const float agentHeight = m_sample->getAgentHeight();
	const float agentClimb  = m_sample->getAgentClimb();

	drawPosition(gBuildbeginPos,agentRadius, agentHeight, agentClimb, startCol);
}

void ChosePositionTool::handleRenderOverlay(double* proj, double* model, int* view)
{}



void ChosePositionTool::drawPosition(const float* pos, float r, float h, float c, const unsigned int col)
{
	duDebugDraw& dd = m_sample->getDebugDraw();

	dd.depthMask(false);
	unsigned int fcol[6] = { 0 };
	duCalcBoxColors(fcol, col, col);

	dd.begin(DU_DRAW_QUADS);
	duAppendBox(&dd, pos[0] - r, pos[1] + 0.02f, pos[2] - r, pos[0] + r, pos[1] + h, pos[2] + r, fcol);
	
	dd.depthMask(true);
	dd.end();
}
