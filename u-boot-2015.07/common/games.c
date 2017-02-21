#include <common.h>

#define WIN_POINTS 1
#define LOSS_POINTS -10 // -1 doesn't work so well, -5 works well, -10 better
#define DRAW_POINTS 0 
#define MAX_PIECE 9 //the play board max of piece
//how many simulations to run each computer move:
#define SIM_ROUND 99999
// which side the human shall play (MARK_NONE for computer vs computer)
#define HUMAN_PLAYER MARK_X  // MARK_X, MARK_O, or MARK_NONE

enum{ MARK_NONE, MARK_X, MARK_O };

static int __maybe_unused my_atoi(char *c)
{
	int i;
	int res = 0;

	for(i = 0; c[i] != '\0'; ++i)
		res = res*10 + c[i] - '0';
	
	return res;
}

/*
 * The LFSR113 for L'écuyer
 * implementation of rand()
*/
static unsigned int my_rand(void)
{
	unsigned int b;
	static unsigned int z1 = 12345;
	static unsigned int z2 = 12345;
	static unsigned int z3 = 12345;
	static unsigned int z4 = 12345;
   
	b  = ((z1 << 6) ^ z1) >> 13;
	z1 = ((z1 & 4294967294U) << 18) ^ b;
	b  = ((z2 << 2) ^ z2) >> 27; 
	z2 = ((z2 & 4294967288U) << 2) ^ b;
	b  = ((z3 << 13) ^ z3) >> 21;
	z3 = ((z3 & 4294967280U) << 7) ^ b;
	b  = ((z4 << 3) ^ z4) >> 12;
	z4 = ((z4 & 4294967168U) << 13) ^ b;

	return (z1 ^ z2 ^ z3 ^ z4);
}

static int ttt_check_win(int *target)
{
	int *p = target;

	//The condiction of connect as a line
	if (p[0] == p[1] && p[1] == p[2]) return p[2];
	if (p[3] == p[4] && p[4] == p[5]) return p[5];
	if (p[6] == p[7] && p[7] == p[8]) return p[8];
	if (p[0] == p[3] && p[3] == p[6]) return p[6];
	if (p[1] == p[4] && p[4] == p[7]) return p[7];
	if (p[2] == p[5] && p[5] == p[8]) return p[8];
	if (p[0] == p[4] && p[4] == p[8]) return p[8];
	if (p[2] == p[4] && p[4] == p[6]) return p[6];

	return MARK_NONE;
}

static inline void ttt_clear_board(int *board)
{
	memset(board, MARK_NONE, sizeof(int)*MAX_PIECE);
}

static int ttt_board_full(int *board)
{
	int i;

	for(i = 0; i < MAX_PIECE; i++)
		if(board[i] == MARK_NONE) return 0;

	return 1;
}

static void ttt_print_board(int *board)
{
	int i;
	char c;

	printf("-----\n");
	for(i = 0; i < MAX_PIECE; i++){
		switch(board[i]){
			case MARK_X: c = 'X'; break;
			case MARK_O: c = 'O'; break;
			default: c = '.';
		}
		printf("%c", c);

		if ((i+1)%3 == 0)
			printf("    %d %d %d\n", i-2, i-1, i);
		else
			printf(" ");
	}
	printf("-----\n");
}

static int _choose_random_move(int *target)
{
	int i, count = 0;
	int pool[MAX_PIECE] = {0};

	/*
	 * Put the pos with empty into pool,
	 * use rand()%count to get it
	*/
	for(i = 0; i < MAX_PIECE; i++){
		if(target[i] == MARK_NONE)
			pool[count++] = i;
	}

	if(count == 0) return -1; //no move

	return pool[my_rand()%count];
}

static inline int _score_pool_init(int *score_pool, int *board)
{
	int i, has_free_space = 0;

	for(i = 0; i < MAX_PIECE; ++i){
		if(board[i] == MARK_NONE){
			has_free_space = 1;
			score_pool[i] = 0;
		}else{
			score_pool[i] = INT_MIN;
		}
	}
	// there are no best moves--cat's game
	if(!has_free_space) return -1;
	
	return 0;
}

static inline void _print_score(int *score_pool)
{
	int i;

	printf("[scores:");
	for(i = 0; i < MAX_PIECE; i++){
		if (score_pool[i] == INT_MIN)
			printf(" --");
		else
			printf(" %d", score_pool[i]);
	}
	printf("]\n");
}

static inline int _get_max_score(int *score_pool)
{
	int i, max_score = INT_MIN;
	
	for(i = 0; i < MAX_PIECE; ++i){
		if(score_pool[i] > max_score)
			max_score = score_pool[i];
	}
	
	return max_score;
}

static inline int _find_best_step(int *score_pool, int max_score)
{
	int step[MAX_PIECE] = {0};
	int i, count = 0;

	printf("[The best AI moving step(s):");
	for(i = 0; i < MAX_PIECE; i++){
		if(score_pool[i] == max_score){
			step[count++] = i;
			printf(" %d", i);
		}
	}
	printf("]\n");
	
	return step[my_rand()%count];
}

#define SETUP_TMP_BOARD(des, src) memcpy(des, src, sizeof(des))
int ttt_ai_simulate(int currealplayer, int *main_board)
{
	int i, pos, firstmove;
	int score_pool[9], max_score;
	int winner, player;
	int tmp_board[MAX_PIECE];

	if(_score_pool_init(score_pool, main_board) == -1) return -1;

	// Start to simulate...
	for(i = 0; i < SIM_ROUND; i++){
		//XXX: memcpy performance is really BAD!!!
		SETUP_TMP_BOARD(tmp_board, main_board);
		player = currealplayer;
		firstmove = -1;

		while((winner = ttt_check_win(tmp_board)) == MARK_NONE){
			pos = _choose_random_move(tmp_board);
			if(pos == -1){
				winner = MARK_NONE; // cat's game
				break;
			}
			tmp_board[pos] = player;

			if(firstmove == -1)
				firstmove = pos;

			player = (player==MARK_X) ? MARK_O : MARK_X;
		}

		// see if we won on this move:
		if(winner != MARK_NONE){
			if(winner == currealplayer)
				score_pool[firstmove] += WIN_POINTS;
			else
				score_pool[firstmove] += LOSS_POINTS;
		}else{
			score_pool[firstmove] += DRAW_POINTS;
		}
	}
	max_score = _get_max_score(score_pool);
	_print_score(score_pool);
	
	return _find_best_step(score_pool, max_score);
}
#undef SETUP_TMP_BOARD

static int _human_input(int *board)
{
	int move;

	while(1){
		puts("Enter a move (0-8): ");
		move = fgetc(stdin)-48;
		if(move < 0 || move > 8)
			printf("move(%d) is invaild!\n", move);
		else if(board[move] != MARK_NONE)
			printf("move(%d) is already exist!\n", move);
		else
			break;
	}
	
	return move;
}

#define WINNER_IS(winner)	do{\
if(winner == MARK_NONE) printf("Cat's game.\n");\
else if(winner == MARK_X) printf("X wins!\n");\
else printf("O wins!\n");	}while(0)
int play_games(void)
{
	int move;
	int bestpos;
	int winner; 
	int curplayer;
	int main_board[MAX_PIECE];

	ttt_clear_board(main_board);
	curplayer = MARK_X;

	// run a game
	while((winner = ttt_check_win(main_board)) == MARK_NONE)
	{
		if(ttt_board_full(main_board)){
			winner = MARK_NONE; // cat's game--no where else to play
			break;
		}

		ttt_print_board(main_board);
	
		//human play
		if(curplayer == HUMAN_PLAYER){
			move = _human_input(main_board);
			main_board[move] = curplayer;
		 }else{ //computer play
			bestpos = ttt_ai_simulate(curplayer, main_board);
			if(main_board[bestpos] != MARK_NONE){
				puts("main_board[bestpos] != MARK_NONE\n");
				return -1;
			}
			printf("Computer plays: %d\n", bestpos);
			main_board[bestpos] = curplayer;
		}
		//update current player
		curplayer = (curplayer==MARK_X) ? MARK_O : MARK_X;
	}

	ttt_print_board(main_board);

	WINNER_IS(winner);

	return 0;
}
#undef WINNER_IS
