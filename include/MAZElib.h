// MAZElib.h

#define WEST_BIT  3
#define NORTH_BIT 2
#define EAST_BIT  1
#define SOUTH_BIT 0

#define WEST_WALL(w)  ((w >> WEST_BIT)  & 1)
#define NORTH_WALL(w) ((w >> NORTH_BIT) & 1)
#define EAST_WALL(w)  ((w >> EAST_BIT)  & 1)
#define SOUTH_WALL(w) ((w >> SOUTH_BIT) & 1)

