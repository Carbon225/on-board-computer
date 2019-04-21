#ifndef _FSFUNCTIONS_H_
#define _FSFUNCTIONS_H_

#include <stdio.h>
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "FS.h"

namespace FSFunctions {
	const char *m_TAG = "Filesystem";

	void listDir(fs::FS &fs, const char * dirname, uint8_t levels) {
		ESP_LOGI(m_TAG, "Listing directory: %s", dirname);

		File root = fs.open(dirname);
		if(!root){
			ESP_LOGE(m_TAG, "Failed to open directory");
			return;
		}
		if(!root.isDirectory()){
			ESP_LOGE(m_TAG, "Not a directory");
			return;
		}

		File file = root.openNextFile();
		while(file){
			if(file.isDirectory()){
				ESP_LOGI(m_TAG, "  DIR : %s", file.name());
				if(levels){
					listDir(fs, file.name(), levels -1);
				}
			} else {
				ESP_LOGI(m_TAG, "  FILE: %s  SIZE: %zu", file.name(), file.size());
			}
			file = root.openNextFile();
		}
	}

	void createDir(fs::FS &fs, const char * path){
		ESP_LOGD(m_TAG, "Creating Dir: %s", path);
		if(fs.mkdir(path)){
			ESP_LOGD(m_TAG, "Dir created");
		} else {
			ESP_LOGE(m_TAG, "mkdir failed");
		}
	}

	void removeDir(fs::FS &fs, const char * path){
		ESP_LOGD(m_TAG, "Removing Dir: %s", path);
		if(fs.rmdir(path)){
			ESP_LOGD(m_TAG, "Dir removed");
		} else {
			ESP_LOGE(m_TAG, "rmdir failed");
		}
	}

	void readFile(fs::FS &fs, const char * path){
		ESP_LOGD(m_TAG, "Reading file: %s", path);

		File file = fs.open(path);
		if(!file){
			ESP_LOGE(m_TAG, "Failed to open file for reading");
			return;
		}

		ESP_LOGI(m_TAG, "Read from file: ");
		while(file.available()){
			printf("%c", file.read());
		}
		printf("\n");
		file.close();
	}

	void writeFile(fs::FS &fs, const char * path, const char * message){
		ESP_LOGD(m_TAG, "Writing file: %s", path);

		File file = fs.open(path, FILE_WRITE);
		if(!file){
			ESP_LOGE(m_TAG, "Failed to open file for writing");
			return;
		}
		if(file.print(message)){
			ESP_LOGD(m_TAG, "File written");
		} else {
			ESP_LOGE(m_TAG, "Write failed");
		}
		file.close();
	}

	void appendFile(fs::FS &fs, const char * path, const char * message){
		ESP_LOGD(m_TAG, "Appending to file: %s", path);

		File file = fs.open(path, FILE_APPEND);
		if(!file){
			ESP_LOGE(m_TAG, "Failed to open file for appending");
			return;
		}
		if(file.print(message)){
			ESP_LOGD(m_TAG, "Message appended");
		} else {
			ESP_LOGE(m_TAG, "Append failed");
		}
		file.close();
	}

	void renameFile(fs::FS &fs, const char * path1, const char * path2){
		Serial.printf("Renaming file %s to %s\n", path1, path2);
		if (fs.rename(path1, path2)) {
			Serial.println("File renamed");
		} else {
			Serial.println("Rename failed");
		}
	}

	void deleteFile(fs::FS &fs, const char * path){
		Serial.printf("Deleting file: %s\n", path);
		if(fs.remove(path)){
			Serial.println("File deleted");
		} else {
			Serial.println("Delete failed");
		}
	}

	void testFileIO(fs::FS &fs, const char * path){
		File file = fs.open(path);
		 uint8_t buf[512];
		size_t len = 0;
		uint32_t start = millis();
		uint32_t end = start;
		if(file){
			len = file.size();
			size_t flen = len;
			start = millis();
			while(len){
				size_t toRead = len;
				if(toRead > 512){
					toRead = 512;
				}
				file.read(buf, toRead);
				len -= toRead;
			}
			end = millis() - start;
			ESP_LOGI(m_TAG, "%u bytes read for %u ms", flen, end);
			file.close();
		} else {
			ESP_LOGE(m_TAG, "Failed to open file for reading");
		}


		file = fs.open(path, FILE_WRITE);
		if(!file){
			ESP_LOGE(m_TAG, "Failed to open file for writing");
			return;
		}

		size_t i;
		start = millis();
		for(i=0; i<2048; i++){
			file.write(buf, 512);
		}
		end = millis() - start;
		ESP_LOGI(m_TAG, "%u bytes written for %u ms", 2048 * 512, end);
		file.close();
	}
};


#endif /* _FSFUNCTIONS_H_ */
