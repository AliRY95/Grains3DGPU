#ifndef _BOUNDINGVOLUME_HH_
#define _BOUNDINGVOLUME_HH_

#include "ReaderXML.hh"


enum BoundingVolumeType {
  AABB,
  OBB,
  OBC
};

// =============================================================================
/** @brief The class BoundingVolume.

    Bounding volume to convex bodies in single precision.

    @author A.Yazdani - 2024 - Modificiation */
// =============================================================================
class BoundingVolume
{
  public:
    /**@name Constructors */
    //@{
    /** @brief Default constructor */
    BVolume();

    /** @brief Destructeur */
    virtual ~BVolume();
    //@}

    /**@name Virtual methods */
    //@{
    /** @brief Returns the type of the bounding volume */
    virtual BVolumeType getBVolumeType() const = 0;

    // Returns a clone of the OBB
    virtual BVolume* clone() const = 0;

    /** @brief Output operator ( is called by << )
    @param fileOut output stream */
    virtual void writeShape( ostream &fileOut ) const = 0;
    //@}


    /** @name Friend methods */
    //@{
    /** @brief Output operator
    @param f output stream
    @param bvol_ BVolume object */
    friend ostream& operator << ( ostream& fileOut, BVolume const& bvol_ );
    //@}
};


/**@name BVolume : External methods */
//@{
/** @brief Returns whether two bounding volumes are in contact
    @param a 1st bounding volume
    @param b 2nd bounding volume
    @param a2w transformation of the 1st bounding volume
    @param b2w transformation of the 2nd bounding volume */
    bool isContact( BVolume const& a,
                    BVolume const& b,
                    Transform const& a2w, 
                    Transform const& b2w );
//@}
    
#endif
