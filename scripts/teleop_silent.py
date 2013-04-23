import curses

    
def get_user_input():
	stdscr = curses.initscr()
	curses.cbreak()
	stdscr.keypad(1)

	stdscr.addstr(0,10,"Hit 'q' to quit")
	stdscr.refresh()

	key = ''
	rudder_angle = 90
	sail_angle = 90
	incr = 5
	while key != ord('q'):
		key = stdscr.getch()
		stdscr.refresh()
		if key == curses.KEY_UP: 
			sail_angle -= incr
		elif key == curses.KEY_LEFT: 
			rudder_angle -= incr
		elif key == curses.KEY_DOWN: 
			sail_angle += incr
		elif key == curses.KEY_RIGHT: 
			rudder_angle += incr
    
	curses.endwin()
	return (rudder_angle, sail_angle)





if __name__ == "__main__":
	get_user_input()
