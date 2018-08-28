package org.usfirst.frc.team694.util;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.StandardCopyOption;

public class FindPath {
	public File leftCSV, rightCSV;
	
	public FindPath(String nameOfPath) {
		leftCSV = new File("/home/lvuser/Paths/" + nameOfPath + "_left_Jaci.csv");
		rightCSV = new File("/home/lvuser/Paths/" + nameOfPath + "_right_Jaci.csv");
		
		try {
			Files.copy(getClass().getResourceAsStream("/" + nameOfPath + "_left_Jaci.csv"), leftCSV.toPath(), StandardCopyOption.REPLACE_EXISTING);
			Files.copy(getClass().getResourceAsStream("/" + nameOfPath + "_right_Jaci.csv"), rightCSV.toPath(), StandardCopyOption.REPLACE_EXISTING);
		}catch(IOException i) {
			System.out.println("Input error");
		}
	}
}
