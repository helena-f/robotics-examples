
public class SudokuPuzzle {
	int start[][] = { { 9, 0, 0, 1, 0, 0, 0, 0, 5 }, { 0, 0, 5, 0, 9, 0, 2, 0, 1 }, { 8, 0, 0, 0, 4, 0, 0, 0, 0 },
			{ 0, 0, 0, 0, 8, 0, 0, 0, 0 }, { 0, 0, 0, 7, 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 2, 6, 0, 0, 9 },
			{ 2, 0, 0, 3, 0, 0, 0, 0, 6 }, { 0, 0, 0, 2, 0, 0, 9, 0, 0 }, { 0, 0, 1, 9, 0, 4, 5, 7, 0 } };
	int board[][] = new int[9][9];
	int currBoard[][] = { { 9, 0, 0, 1, 0, 0, 0, 0, 5 }, { 0, 0, 5, 0, 9, 0, 2, 0, 1 }, { 8, 0, 0, 0, 4, 0, 0, 0, 0 },
			{ 0, 0, 0, 0, 8, 0, 0, 0, 0 }, { 0, 0, 0, 7, 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 2, 6, 0, 0, 9 },
			{ 2, 0, 0, 3, 0, 0, 0, 0, 6 }, { 0, 0, 0, 2, 0, 0, 9, 0, 0 }, { 0, 0, 1, 9, 0, 4, 5, 7, 0 } };

	// constructor
	public SudokuPuzzle(int[][] board) {
		this.board = new int[9][9];
		for (int i = 0; i < 9; i++) {
			for (int j = 0; j < 9; j++) {
				this.board[i][j] = board[i][j];
			}
		}
	}

	public boolean isInRow(int row, int value) {
		for (int i = 0; i < 9; i++) {
			if (board[row][i] == value)
				return true;
		}
		return false;
	}

	public boolean isInCol(int col, int value) {
		for (int i = 0; i < 9; i++) {
			if (board[i][col] == value)
				return true;
		}
		return false;
	}

	public boolean isInBox(int row, int col, int value) {
		int r = row - row % 3;
		int c = col - col % 3;

		for (int i = r; i < r; i++) {
			for (int j = c; j < c; j++) {
				if (board[i][j] == value)
					return true;
			}
		}
		return false;
	}

	// returns true if the values in the puzzle do not violate the restrictions
	public boolean isOk(int row, int col, int value) {
		return !isInRow(row, value) && !isInCol(col, value) && !isInBox(row, col, value);
	}

	public boolean solve() {
		for (int row = 0; row < 9; row++) {
			for (int col = 0; col < 9; col++) {
				if (board[row][col] == 0) {
					for (int number = 1; number <= 9; number++) {
						if (isOk(row, col, number)) {
							board[row][col] = number;

							if (solve()) {
								return true;
							} else {
								board[row][col] = 0;
							}
						}
					}
					return false;
				}
			}
		}
		return true;
	}

	// sets the given square to the given value; the value can be changed later by
	// another call to addGuess
	public int[][] addGuess(int row, int col, int value) {
		for (int i = 0; i < 9; i++) {
			for (int j = 0; j < 9; j++) {
				if (i == row && j == col) {
					currBoard[row][col] = value;
				}
			}
		}
		return currBoard;
	}

	// returns true if the values in the puzzle do not violate the restrictions
	public boolean checkPuzzle(int row, int col, int value) {
		boolean[] arr = getAllowedValues(row, col);
		if (arr[value - 1])
			return true;
		else
			return false;
	}

	// returns the value in the given square
	public int getValueIn(int row, int col) {
		for (int i = 0; i < 9; i++) {
			for (int j = 0; j < 9; j++) {
				if (i == row && j == col) {
					return currBoard[i][j];
				}
			}
		}
		return 0;
	}

	// returns a one-dimensional array of nine booleans, each of which corresponds
	// to a digit and is true if the digit can be placed in the given square without
	// violating the restrictions
	public boolean[] getAllowedValues(int row, int col) {
		boolean[] arr = new boolean[9];
		for (int k = 1; k <= 9; k++) {
			arr[k-1] = !(values(row, col, k));

		}
		return arr;

	}

	public boolean values(int row, int col, int k) {
		for (int i = 0; i < 9; i++) {
			if (k == currBoard[row][i] || k == currBoard[i][col]) {
				return true;
			}
		}
		return false;
	}
	
	
	// returns true if every square has a value
	public boolean isFull() {
		for (int i = 0; i < 9; i++) {
			for (int j = 0; j < 9; j++) {
				if (currBoard[i][j] == 0) {
					return false;
				}
			}
		}
		return true;
	}

	// changes all of the nonpermanent squares back to blanks (0s)
	public int[][] reset() {
		return start;
	}

	// returns a string representation of the puzzle that can be printed
	public void toStringDisplay() {
		for (int i = 0; i < 9; i++) {
			for (int j = 0; j < 9; j++) {
				System.out.print(" " + currBoard[i][j]);
			}
			System.out.println();
		}
		System.out.println();
	}
}
