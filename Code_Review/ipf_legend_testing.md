# IPF Color Legend Testing

- Each Laue class has a subsection below
- Each Laue class has 2 options: "NH" and "TSL"

## After IPF Legends are all matching

- Create a square grid of Euler Angles that can reasonable represent a wide range of orientations
- Treat that Square Grid like a normal EBSD Scan
- Render it with the variations of the "IPF Colors" using both MTEX and EbsdLib
- Compare the images that are produced. They should be nearly exactly the same.


## 1

### TSL Comparison

- These will never match because MTEX uses a polar coordinate system instead of TSL's etc/chi equations
- [ ] EbsdLib Per Pixel and MTEX Match
- [ ] Ebsdlib Gridded and MTEX Match


### NH Gridded Comparison

- [ ] EbsdLib Per Pixel and MTEX Match
- [ ] Ebsdlib Gridded and MTEX Match


## 2

### TSL Comparison


- [X] EbsdLib Per Pixel and MTEX Match
- [X] Ebsdlib Gridded and MTEX Match

### NH Gridded Comparison

- [ ] EbsdLib Per Pixel and MTEX Match
- [ ] Ebsdlib Gridded and MTEX Match


## 3

### TSL Comparison


- [X] EbsdLib Per Pixel and MTEX Match
- [X] Ebsdlib Gridded and MTEX Match

### NH Gridded Comparison

- [ ] EbsdLib Per Pixel and MTEX Match
- [ ] Ebsdlib Gridded and MTEX Match

## 4

### TSL Comparison


- [X] EbsdLib Per Pixel and MTEX Match
- [X] Ebsdlib Gridded and MTEX Match

### NH Gridded Comparison

- [ ] EbsdLib Per Pixel and MTEX Match
- [ ] Ebsdlib Gridded and MTEX Match

## 6

### TSL Comparison


- [X] EbsdLib Per Pixel and MTEX Match
- [X] Ebsdlib Gridded and MTEX Match

### NH Gridded Comparison

- [ ] EbsdLib Per Pixel and MTEX Match
- [ ] Ebsdlib Gridded and MTEX Match


## 23

### TSL Comparison


- [X] EbsdLib Per Pixel and MTEX Match
- [X] Ebsdlib Gridded and MTEX Match

### NH Gridded Comparison

- [ ] EbsdLib Per Pixel and MTEX Match
- [ ] Ebsdlib Gridded and MTEX Match

## 32

### TSL Comparison


- [X] EbsdLib Per Pixel and MTEX Match
- [X] Ebsdlib Gridded and MTEX Match

### NH Gridded Comparison

- [ ] EbsdLib Per Pixel and MTEX Match
- [ ] Ebsdlib Gridded and MTEX Match

## 222

### TSL Comparison


- [X] EbsdLib Per Pixel and MTEX Match
- [X] Ebsdlib Gridded and MTEX Match

### NH Gridded Comparison

- [ ] EbsdLib Per Pixel and MTEX Match
- [ ] Ebsdlib Gridded and MTEX Match

## 422

### TSL Comparison


- [X] EbsdLib Per Pixel and MTEX Match
- [X] Ebsdlib Gridded and MTEX Match

### NH Gridded Comparison

- [ ] EbsdLib Per Pixel and MTEX Match
- [ ] Ebsdlib Gridded and MTEX Match

## 432

### TSL Comparison


- [X] EbsdLib Per Pixel and MTEX Match
- [X] Ebsdlib Gridded and MTEX Match

### NH Gridded Comparison

- [ ] EbsdLib Per Pixel and MTEX Match
- [ ] Ebsdlib Gridded and MTEX Match

## 622

### TSL Comparison

- EbsdLib there is a single vertical pixel row that should not be in the image
- EbsdLib needs to be flipped vertically (rotate around the X Axis) and then have the labels updated to match MTEX
- MTEX is using the TSL color table but using the Gridded rendering scheme, can EbsdLib do the same, at least for the comparison?


- [X] EbsdLib Per Pixel and MTEX Match
- [X] Ebsdlib Gridded and MTEX Match

### NH Gridded Comparison

- [ ] EbsdLib Per Pixel and MTEX Match
- [ ] Ebsdlib Gridded and MTEX Match
