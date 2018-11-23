from PIL import Image
 
image = Image.open('bild.png') 
image = image.convert('RGB')
 
width ,height = image.size
array = []
 
for y in range(0, height): 
    for x in range(0, width):
 
        rgb = image.getpixel((x,y))
	#left = x-1
	#high = y-1
	#if(left == -1): 
	#	left = 0
	#if(high == -1):
	#	high = 0
	#black_left = image.getpixel((left,y))
	#black_high = image.getpixel((x,high))
        #if (rgb == (255,255,255) and black_left == (0,0,0) and black_high == (0,0,0)):   
	if( (x,y) == (218,251) or (x,y) == (430,246) or (x,y) == (172,302) or (x,y) == (459,290) or (x,y) == (119,381) or (x,y) == (517,365) ): 
        	array.append((x,y))
print array
