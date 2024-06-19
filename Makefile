pk01_dma: pk01_dma.o pk01_dma_utils.o
	gcc pk01_dma.o pk01_dma_utils.o -o pk01_dma

pk01_dma.o: pk01_dma.c
	gcc -c pk01_dma.c 

pk01_dma_utils.o: pk01_dma_utils.c 
	gcc -c pk01_dma_utils.c 

filter.o: filter.c 
	gcc -c filter.c 

clean:
	rm -f ./*.o ./pk01_dma

