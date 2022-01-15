# Optimal Reciprocal Collision Avoidance

This is a port of https://github.com/snape/RVO2 and https://github.com/snape/RVO2-3D to Kotlin.

You therefore could compile this library to JavaScript and Java.

I also changed floats to doubles, because I intend to use this libary in my game engine xD.
Changing it back should be easy.

## Dependencies

The only dependency is [JOML](https://github.com/JOML-CI/JOML), but it could be replaced easily.

## Functionality

It feels like I still have bugs in the library, and I wouldn't call it production ready, but it might work good enough for you :).
If you find a typo/bug fix, write me :).

## Java-Optimizations

While allocations are cheap, they can inflate the memory usage or cause GC-lag-spikes,
so I've tried to minimize allocations a bit.

## Parallelization

Currently, there is no parallelization, but it shouldn't be too difficult to add.