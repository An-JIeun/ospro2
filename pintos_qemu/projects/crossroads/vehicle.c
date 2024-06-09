
#include <stdio.h>

#include "threads/thread.h"
#include "threads/synch.h"
#include "projects/crossroads/vehicle.h"
#include "projects/crossroads/map.h"
#include "projects/crossroads/ats.h"
#include "projects/crossroads/graph.h"
int vehicle_count;

struct vehicle_info* vehicles = (struct vehicle_info *)vehicle_list;
/* path. A:0 B:1 C:2 D:3 */
const struct position vehicle_path[4][4][12] = {
	/* from A */ {
		/* to A */
		{{4,0},{4,1},{4,2},{4,3},{4,4},{3,4},{2,4},{2,3},{2,2},{2,1},{2,0},{-1,-1},},
		/* to B */
		{{4,0},{4,1},{4,2},{5,2},{6,2},{-1,-1},},
		/* to C */
		{{4,0},{4,1},{4,2},{4,3},{4,4},{4,5},{4,6},{-1,-1},},
		/* to D */
		{{4,0},{4,1},{4,2},{4,3},{4,4},{3,4},{2,4},{1,4},{0,4},{-1,-1},}
	},
	/* from B */ {
		/* to A */
		{{6,4},{5,4},{4,4},{3,4},{2,4},{2,3},{2,2},{2,1},{2,0},{-1,-1},},
		/* to B */
		{{6,4},{5,4},{4,4},{3,4},{2,4},{2,3},{2,2},{3,2},{4,2},{5,2},{6,2},{-1,-1},},
		/* to C */
		{{6,4},{5,4},{4,4},{4,5},{4,6},{-1,-1},},
		/* to D */
		{{6,4},{5,4},{4,4},{3,4},{2,4},{1,4},{0,4},{-1,-1},}
	},
	/* from C */ {
		/* to A */
		{{2,6},{2,5},{2,4},{2,3},{2,2},{2,1},{2,0},{-1,-1},},
		/* to B */
		{{2,6},{2,5},{2,4},{2,3},{2,2},{3,2},{4,2},{5,2},{6,2},{-1,-1},},
		/* to C */
		{{2,6},{2,5},{2,4},{2,3},{2,2},{3,2},{4,2},{4,3},{4,4},{4,5},{4,6},{-1,-1},},
		/* to D */
		{{2,6},{2,5},{2,4},{1,4},{0,4},{-1,-1},}
	},
	/* from D */ {
		/* to A */
		{{0,2},{1,2},{2,2},{2,1},{2,0},{-1,-1},},
		/* to B */
		{{0,2},{1,2},{2,2},{3,2},{4,2},{5,2},{6,2},{-1,-1},},
		/* to C */
		{{0,2},{1,2},{2,2},{3,2},{4,2},{4,3},{4,4},{4,5},{4,6},{-1,-1},},
		/* to D */
		{{0,2},{1,2},{2,2},{3,2},{4,2},{4,3},{4,4},{3,4},{2,4},{1,4},{0,4},{-1,-1},}
	}
};

static int is_position_outside(struct position pos)
{
	return (pos.row == -1 || pos.col == -1);
}

/*additional infos*/
bool RAG[20][20];

/*Deadlock Detection function*/
bool detect_cycle_unit(int v, bool visted[], bool rec_stack[]){
	if(!visted[v]){
		visted[v] = true;
		rec_stack[v] = true;

		for (int i = 0;i < vehicle_count;i++) {
			if (RAG[v][i] && !visted[i] && detect_cycle_unit(i, visted, rec_stack)) {
				return true;
			}
			else if (rec_stack[i]){
				return true;
			}
		}
	}
	rec_stack[v] = false;
	return false;
}

/* nested deteciton functions */
bool detect_cycle(){
	bool visted[20];
	bool rec_stack[20];

	for (int i = 0;i < vehicle_count;i++) {
		if (detect_cycle_unit(i, visted, rec_stack)){
			return true;
		}
	}
	return false;

}


/* return 0:termination, 1:success, -1:fail */
static int try_move(int start, int dest, int step, struct vehicle_info *vi)
{
	struct position pos_cur, pos_next;

	pos_next = vehicle_path[start][dest][step];
	pos_cur = vi->position;

	if (vi->state == VEHICLE_STATUS_RUNNING) {
		/* check termination */
		if (is_position_outside(pos_next)) {
			/* actual move */
			vi->position.row = vi->position.col = -1;
			/* release previous */
			lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
			return 0;
		}
	}

	/* lock next position */
	lock_acquire(&vi->map_locks[pos_next.row][pos_next.col]);
	if (vi->state == VEHICLE_STATUS_READY) {
		/* start this vehicle */
		vi->state = VEHICLE_STATUS_RUNNING;
	} else {
		/* release current position */
		lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
	}
	/* update position */
	vi->position = pos_next;
	
	/* 자원할당 그래프 업데이트 */
	for (int i = 0; i < vehicle_count; i++){
		if (vehicles[i].position.row == pos_next.row && vehicles[i].position.col == pos_next.col){
			RAG[vi - vehicles][i] = false;
			if (detect_cycle()){
				RAG[vi - vehicles][i] = true;
				lock_release(&vi->map_locks[pos_next.row][pos_next.col]);
				return -1; // Deadlock 발생
			}
		}
	}



	return 1;
}

void init_on_mainthread(int thread_cnt){
	/* Called once before spawning threads */
	vehicle_count = thread_cnt; // theads length
}

void vehicle_loop(void *_vi)
{
	int res;
	int start, dest, step;

	struct vehicle_info *vi = _vi;

	start = vi->start-'A';
	dest = vi->dest - 'A';

	vi->position.row = vi->position.col = -1;
	vi->state = VEHICLE_STATUS_READY;

	step = 0;
	while (1) {
		/* vehicle main code */
		res = try_move(start, dest, step, vi);
		if (res == 1) {
			step++;
		}

		/*Deadlock 발생 대처*/
		if (res == -1)
		{
			lock_release(&vi->map_locks[vi->position.row][vi->position.col]);
			step = 0;
			vi->position.row = vi->position.col = -1;
			vi->state = VEHICLE_STATUS_READY;
			continue;
		}
		

		/* termination condition. */ 
		if (res == 0) {
			break;
		}

		/* unitstep change! */
		unitstep_changed();
	}	

	/* status transition must happen before sema_up */
	vi->state = VEHICLE_STATUS_FINISHED;
}
