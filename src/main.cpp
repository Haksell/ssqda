#include "gladiator.h"
#include <algorithm>
#include <chrono>
#undef abs

// TODO: all pairs shortest path
// TODO: use previous wheel speed and update
// TODO: go backwards when appropriate

#define SIZE 12
#define GOTO_ANGLE 0.2
#define DFS_DEPTH 7
#define M_TAU 6.283185307179586
#define M_HALF_PI 1.5707963267948966
#define EXPONENTIONAL_EXPONENT 0.9

float EXPONENTS[DFS_DEPTH] = {1.0};

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

int** stack;
uint8_t** possessions;

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
	static const float vlimit = 0.7;
	static const float epsilon = 0.07;
	double consvl, consvr;
	double dx = toPos.x - fromPos.x;
	double dy = toPos.y - fromPos.y;
	double d = sqrt(dx * dx + dy * dy);

	if (d > epsilon) {
		double rho = atan2(dy, dx);
		double angle = reductionAngle(rho - fromPos.a);
		double consw = clamp(angle, -wlimit, wlimit);
		double consv = clamp(clamp(d, 0.3, 3) * cos(angle), -vlimit, vlimit);
		double speedLimit = gladiator->robot->getData().speedLimit;
		consvl = clamp(consv - consw, -1.0, 1.0) * speedLimit;
		consvr = clamp(consv + consw, -1.0, 1.0) * speedLimit;
	} else {
		consvl = 0.0;
		consvr = 0.0;
	}
	gladiator->control->setWheelSpeed(WheelAxis::LEFT, consvl, false);
	gladiator->control->setWheelSpeed(WheelAxis::RIGHT, consvr, false);
}

void reset() {
	frameCount = 0;
	currentTimeBlock = 0;
	minIdx = 0;
	maxIdx = SIZE - 1;
	startTime = std::chrono::high_resolution_clock::now();
	squareSize = gladiator->maze->getSquareSize();
	teamId = gladiator->robot->getData().teamId;
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
	for (size_t i = 1; i < DFS_DEPTH; ++i) {
		EXPONENTS[i] = EXPONENTS[i - 1] * EXPONENTIONAL_EXPONENT;
	}
	stack = (int**)malloc(DFS_DEPTH * sizeof(int*));
	for (size_t i = 0; i < DFS_DEPTH; ++i)
		stack[i] = (int*)malloc(2 * sizeof(int));
	possessions = (uint8_t**)malloc(SIZE * sizeof(uint8_t*));
	for (size_t i = 0; i < SIZE; ++i)
		possessions[i] = (uint8_t*)malloc(SIZE * sizeof(uint8_t));
	gladiator = new Gladiator();
	gladiator->game->onReset(&reset);
}

void dfs(int** stack, size_t depth, float score, float* bestScore, int* bestX, int* bestY,
		 uint8_t** possessions) {
	static int neighbors[5][3] = {
		{0, 0, ROCKET}, {0, 1, NORTH}, {1, 0, EAST}, {0, -1, SOUTH}, {-1, 0, WEST},
	};
	if (score > *bestScore) {
		*bestScore = score;
		*bestX = stack[1][0];
		*bestY = stack[1][1];
	}
	if (depth == DFS_DEPTH) return;
	for (size_t i = 0; i < 5; ++i) {
		int x = stack[depth - 1][0] + neighbors[i][0];
		int y = stack[depth - 1][1] + neighbors[i][1];
		t_dir dir = (t_dir)neighbors[i][2];
		if (x < minIdx || x > maxIdx || y < minIdx || y > maxIdx) continue;
		float addScore =
			(possessions[y][x] & 4) * 4.0 +
			(possessions[y][x] == teamId ? 0.1
			 : possessions[y][x] == 0	 ? 1
										 : 2) -
			10 * ((dir == NORTH && !maze[y][x][SOUTH]) || (dir == EAST && !maze[y][x][WEST]) ||
				  (dir == SOUTH && !maze[y][x][NORTH]) || (dir == WEST && !maze[y][x][EAST])) -
			0.5 * (dir == ROCKET);
		float newScore = score + addScore * EXPONENTS[depth];
		uint8_t prev = possessions[y][x];
		possessions[y][x] = teamId;
		stack[depth][0] = x;
		stack[depth][1] = y;
		dfs(stack, depth + 1, newScore, bestScore, bestX, bestY, possessions);
		possessions[y][x] = prev;
	}
}

void getNextMove(int x, int y, int* bestX, int* bestY) {
	for (size_t my = 0; my < SIZE; my++) {
		for (size_t mx = 0; mx < SIZE; mx++) {
			const MazeSquare* mazeSquare = gladiator->maze->getSquare(mx, my);
			possessions[my][mx] = mazeSquare->possession | mazeSquare->coin.value << 2;
		}
	}
	stack[0][0] = x;
	stack[0][1] = y;
	float bestScore = 0.0f;
	dfs((int**)stack, 1, 0, &bestScore, bestX, bestY, (uint8_t**)possessions);
}

void loop() {
	if (gladiator->game->isStarted() && gladiator->robot->getData().lifes) {
		if (frameCount == 0) {
			delay(69);
		}
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
		int bestX = SIZE >> 1;
		int bestY = SIZE >> 1;
		getNextMove(x, y, &bestX, &bestY);
		float targetX = ((float)bestX + 0.5f) * squareSize;
		float targetY = ((float)bestY + 0.5f) * squareSize;
		gladiator->log("%.2f %.2f", targetX, targetY);
		Position goal{targetX, targetY, 0};
		goTo(myPosition, goal);
		delay(5);
	}
}