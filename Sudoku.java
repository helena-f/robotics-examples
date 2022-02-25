import java.util.*;

public class Sudoku {
	
	public static void main ( String [] args) {
		int start [][] = {
				{9,0,0,1,0,0,0,0,5},		
				{0,0,5,0,9,0,2,0,1},
				{8,0,0,0,4,0,0,0,0},
				{0,0,0,0,8,0,0,0,0},
				{0,0,0,7,0,0,0,0,0},
				{0,0,0,0,2,6,0,0,9},
				{2,0,0,3,0,0,0,0,6},
				{0,0,0,2,0,0,9,0,0},
				{0,0,1,9,0,4,5,7,0}
		};
		
		SudokuPuzzle puzzle = new SudokuPuzzle(start);
		Scanner scanner = new Scanner(System.in);
		int col, row, value = 0;
		
		System.out.println("Welcome to Sudoku Puzzle!");
		System.out.println("Solve the following puzzle:");
		puzzle.toStringDisplay();
		
		boolean playerWin = false;
		
		while (!playerWin) {
			System.out.println("Column number: ");
			col = scanner.nextInt() - 1;
			System.out.println("Row number: ");
			row = scanner.nextInt() - 1;
			System.out.println("Value: ");
			value = scanner.nextInt();
			
			if ( start[row][col] == 0 && puzzle.checkPuzzle(row, col, value)) {
				puzzle.addGuess(row, col, value);
				puzzle.toStringDisplay();
			} else if (start[row][col] != 0) {
				System.out.println("Static value. Please select another square.");
			} else if (!puzzle.checkPuzzle(row, col, value)) {
				System.out.println("Already used number in column, row, or square. Try again.");
			}
		
		
			
			if (puzzle.isFull()) {
				playerWin = true;
				System.out.println("You win! Play Again? (y/n)");
				String str = scanner.next();
				if (str.equals("y")) puzzle.reset(); playerWin = false; 
			}		
		}
		
		
		if (puzzle.solve()) {
			System.out.println("Solved: ");
			puzzle.toStringDisplay();
		}
		
	}

}
