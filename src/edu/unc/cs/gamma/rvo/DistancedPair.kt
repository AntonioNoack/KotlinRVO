package edu.unc.cs.gamma.rvo

class DistancedPair<Type>(var distSq: Double, var instance: Type) {

    fun set(other: DistancedPair<Type>) {
        distSq = other.distSq
        instance = other.instance
    }

    fun set(f: Double, o: Type) {
        distSq = f
        instance = o
    }

}