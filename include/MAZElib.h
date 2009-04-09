// MAZElib.h

#define WEST_BIT  3
#define NORTH_BIT 2
#define EAST_BIT  1
#define SOUTH_BIT 0

#define HAS_WEST_WALL(w)  ((w >> WEST_BIT)  & 1)
#define HAS_NORTH_WALL(w) ((w >> NORTH_BIT) & 1)
#define HAS_EAST_WALL(w)  ((w >> EAST_BIT)  & 1)
#define HAS_SOUTH_WALL(w) ((w >> SOUTH_BIT) & 1)

#define SET_WEST_WALL(w)  (w = (w | (1 << WEST_BIT)))
#define SET_NORTH_WALL(w) (w = (w | (1 << NORTH_BIT)))
#define SET_EAST_WALL(w)  (w = (w | (1 << EAST_BIT)))
#define SET_SOUTH_WALL(w) (w = (w | (1 << SOUTH_BIT)))

#define CLEAR_WEST_WALL(w)  (w = (w & ~(1 << WEST_BIT)))
#define CLEAR_NORTH_WALL(w) (w = (w & ~(1 << NORTH_BIT)))
#define CLEAR_EAST_WALL(w)  (w = (w & ~(1 << EAST_BIT)))
#define CLEAR_SOUTH_WALL(w) (w = (w & ~(1 << SOUTH_BIT)))

#define COUNT_WALLS(w)  (HAS_WEST_WALL(w) + HAS_EAST_WALL(w) + HAS_NORTH_WALL(w) + HAS_SOUTH_WALL(w))

int Move_To_Next(api_HANDLES_t *, FilterHandles_t *, pidHandles_t *, int);
int What_Do_I_See(api_HANDLES_t * dev, FilterHandles_t * filt, double * N, double * S, double * E, double * W);
