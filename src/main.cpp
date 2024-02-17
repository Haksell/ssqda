#include "gladiator.h"
#include <algorithm>
#include <chrono>
#undef abs

// TODO: all pairs shortest path
// TODO: use previous wheel speed and update

#define SIZE 12
#define GOTO_ANGLE 0.2
#define DFS_SIZE 6
#define M_TAU 6.283185307179586
#define M_HALF_PI 1.5707963267948966

typedef enum e_dir { NORTH = 0, EAST, SOUTH, WEST, ROCKET } t_dir;

Gladiator* gladiator;
bool maze[SIZE][SIZE][4];
uint8_t teamId = 0;
std::chrono::high_resolution_clock::time_point startTime;
int frameCount = 0;
int currentTimeBlock = 0;
int minIdx = 0;
int maxIdx = SIZE - 1;
float squareSize = 0.0f;

double reductionAngle(double x) {
	x = fmod(x + PI, 2 * PI);
	if (x < 0) x += 2 * PI;
	return x - PI;
}

void printMaze() {
	for (int y = 0; y < SIZE; ++y) {
		gladiator->log("%02d %02d %02d %02d %02d %02d %02d %02d %02d %02d %02d %02d",
					   maze[y][0][NORTH] | maze[y][0][EAST] << 1 | maze[y][0][SOUTH] << 2 |
						   maze[y][0][WEST] << 3,
					   maze[y][1][NORTH] | maze[y][1][EAST] << 1 | maze[y][1][SOUTH] << 2 |
						   maze[y][1][WEST] << 3,
					   maze[y][2][NORTH] | maze[y][2][EAST] << 1 | maze[y][2][SOUTH] << 2 |
						   maze[y][2][WEST] << 3,
					   maze[y][3][NORTH] | maze[y][3][EAST] << 1 | maze[y][3][SOUTH] << 2 |
						   maze[y][3][WEST] << 3,
					   maze[y][4][NORTH] | maze[y][4][EAST] << 1 | maze[y][4][SOUTH] << 2 |
						   maze[y][4][WEST] << 3,
					   maze[y][5][NORTH] | maze[y][5][EAST] << 1 | maze[y][5][SOUTH] << 2 |
						   maze[y][5][WEST] << 3,
					   maze[y][6][NORTH] | maze[y][6][EAST] << 1 | maze[y][6][SOUTH] << 2 |
						   maze[y][6][WEST] << 3,
					   maze[y][7][NORTH] | maze[y][7][EAST] << 1 | maze[y][7][SOUTH] << 2 |
						   maze[y][7][WEST] << 3,
					   maze[y][8][NORTH] | maze[y][8][EAST] << 1 | maze[y][8][SOUTH] << 2 |
						   maze[y][8][WEST] << 3,
					   maze[y][9][NORTH] | maze[y][9][EAST] << 1 | maze[y][9][SOUTH] << 2 |
						   maze[y][9][WEST] << 3,
					   maze[y][10][NORTH] | maze[y][10][EAST] << 1 | maze[y][10][SOUTH] << 2 |
						   maze[y][10][WEST] << 3,
					   maze[y][11][NORTH] | maze[y][11][EAST] << 1 | maze[y][11][SOUTH] << 2 |
						   maze[y][11][WEST] << 3);
	}
}

double clamp(double x, double mini, double maxi) { return x < mini ? mini : x > maxi ? maxi : x; }

void goTo(Position fromPos, Position toPos) {
	static const float wlimit = 0.4;
	static const float vlimit = 0.6;
	static const float epsilon = 0.07;
	double consvl, consvr;
	double dx = toPos.x - fromPos.x;
	double dy = toPos.y - fromPos.y;
	double d = sqrt(dx * dx + dy * dy);

	if (d > epsilon) {
		double rho = atan2(dy, dx);
		double angle = reductionAngle(rho - fromPos.a);
		double consw = clamp(12 * angle, -wlimit, wlimit);
		double consv = clamp(d * cos(angle), -vlimit, vlimit);
		consvl = consv - consw;
		consvr = consv + consw;
	} else {
		consvl = 0;
		consvr = 0;
	}
	gladiator->control->setWheelSpeed(WheelAxis::RIGHT, consvr, false);
	gladiator->control->setWheelSpeed(WheelAxis::LEFT, consvl, false);
}

void reset() {
	startTime = std::chrono::high_resolution_clock::now();
	frameCount = 0;
	squareSize = gladiator->maze->getSquareSize();
	teamId = gladiator->robot->getData().teamId;
	gladiator->log("%d", teamId);
	for (int y = 0; y < SIZE; ++y) {
		for (int x = 0; x < SIZE; ++x) {
			const MazeSquare* mazeSquare = gladiator->maze->getSquare(x, y);
			maze[y][x][NORTH] = !!mazeSquare->northSquare;
			maze[y][x][EAST] = !!mazeSquare->eastSquare;
			maze[y][x][SOUTH] = !!mazeSquare->southSquare;
			maze[y][x][WEST] = !!mazeSquare->westSquare;
		}
	}
	// printMaze();
}

void setup() {
	gladiator = new Gladiator();
	gladiator->game->onReset(&reset);
}

int getScore(int x, int y, t_dir dir) {
	if (x < minIdx || y < minIdx || x > maxIdx || y > maxIdx) return -1;
	if ((dir == NORTH && !maze[y][x][SOUTH]) || (dir == EAST && !maze[y][x][WEST]) ||
		(dir == SOUTH && !maze[y][x][NORTH]) || (dir == WEST && !maze[y][x][EAST]))
		return -1;
	const MazeSquare* mazeSquare = gladiator->maze->getSquare(x, y);
	if (dir == ROCKET && mazeSquare->possession == teamId) return -1;
	return mazeSquare->coin.value * 3 + (mazeSquare->possession == teamId ? 0
										 : mazeSquare->possession == 0	  ? 1
																		  : 2);
}

void loop() {
	if (gladiator->game->isStarted()) {
		++frameCount;
		if ((frameCount & 63) == 0) {
			auto currentTime = std::chrono::high_resolution_clock::now();
			auto deltaTime =
				std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - startTime)
					.count();
			int newTimeBlock = (deltaTime + 2000) / 20000;
			if (newTimeBlock != currentTimeBlock) {
				currentTimeBlock = newTimeBlock;
				++minIdx;
				--maxIdx;
			}
		}
		if (gladiator->weapon->canLaunchRocket()) gladiator->weapon->launchRocket();
		Position myPosition = gladiator->robot->getData().position;
		int x = (int)(myPosition.x / squareSize);
		int y = (int)(myPosition.y / squareSize);
		int neighbors[5][3] = {
			{x, y, ROCKET},	   {x, y + 1, NORTH}, {x + 1, y, EAST},
			{x, y - 1, SOUTH}, {x - 1, y, WEST},
		};
		int bestX = -1;
		int bestY = -1;
		int bestScore = -1;
		for (uint8_t i = 0; i < 5; ++i) {
			int score = getScore(neighbors[i][0], neighbors[i][1], (t_dir)neighbors[i][2]);
			if (score > bestScore) {
				bestScore = score;
				bestX = neighbors[i][0];
				bestY = neighbors[i][1];
			}
		}
		if (bestScore == -1) {
			bestX = SIZE >> 1;
			bestY = SIZE >> 1;
			for (int tryY = minIdx; tryY <= maxIdx; ++tryY) {
				for (int tryX = minIdx; tryX <= maxIdx; ++tryX) {
					int score = getScore(tryX, tryY, ROCKET);
					if (score > bestScore) {
						bestScore = score;
						bestX = tryX;
						bestY = tryY;
					}
				}
			}
		}
		float targetX = ((float)bestX + 0.5f) * squareSize;
		float targetY = ((float)bestY + 0.5f) * squareSize;
		Position goal{targetX, targetY, 0};
		goTo(myPosition, goal);
		delay(5);
	}
}
