package org.usfirst.frc.team694.util.profiles;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
import java.util.Scanner; 

public class FindPath {
	public File leftCSV, rightCSV;
	
	public FindPath(String nameOfPath) {
		leftCSV = new File("/home/lvuser/Paths/" + nameOfPath + "_left_Jaci.csv");
		rightCSV = new File("/home/lvuser/Paths/" + nameOfPath + "_right_Jaci.csv");
		
		try {
			InputStream leftStream = getClass().getResourceAsStream("/" + nameOfPath + "_left_Jaci.csv");
			streamToFile(leftStream, leftCSV); 
			InputStream rightStream = getClass().getResourceAsStream("/" + nameOfPath + "_right_Jaci.csv");
			streamToFile(rightStream, rightCSV); 
		}catch(Exception e) {
			e.printStackTrace();
		}
	}
	
	private void streamToFile(InputStream is, File file) {
		try {
			Scanner scanner = new Scanner(is);
			FileWriter fWriter = new FileWriter(file);
			BufferedWriter bWriter = new BufferedWriter(fWriter);
			
			boolean hasNext = scanner.hasNext(); 
			while(hasNext) {
				bWriter.write(scanner.next());
				bWriter.flush();
				hasNext = scanner.hasNext(); 
				if(hasNext) bWriter.newLine();
			}
			
			bWriter.close();
			scanner.close();
			is.close();
		}catch(IOException i) {
			System.out.println("Input Error");
			i.printStackTrace();
		}
	}
}
