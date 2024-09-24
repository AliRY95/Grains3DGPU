#ifndef _INSERTION_HH_
#define _INSERTION_HH_


#include <variant>
#include "Transform3.hh"
#include "Kinematics.hh"
#include "ReaderXML.hh"


/** @name Enumerations */
//@{    
/** @brief Various supported insertion types */
enum InsertionType {
    RANDOM,
    FILE,
    CONSTANT,
    DEFAULT
};

/** @brief Various supported insertion modes */
enum InsertionMode {
    INITIAL,
    OVERTIME
};

/** \brief info required for comping up with an insertion position. It can be 
either a value (T) that is used as the seed for random generator algorithm, a 
string (std::string) that is used as the pathToFile, a 3d vector (vector3<T>) 
for constant values, and nothing (void) in case the default insertion option is 
desired. */
using InsertionInfo = std::variant<T, std::string, Vector3<T>, void>;
//@}


// =============================================================================
/** @brief The class Insertion.

    This class provides funtionalities to insert components in the simulation.

    @author A.YAZDANI - 2024 - Construction */
// =============================================================================
template <typename T>
class Insertion
{
	protected:
        /** @name Parameters */
        //@{
        /** \brief insertion type for position */
        InsertionType m_positionType;
        /** \brief insertion type for orientation */
        InsertionType m_orientationType;
        /** \brief insertion type for translational velocity */
        InsertionType m_translationalVelType;
        /** \brief insertion type for angular velocity */
        InsertionType m_angularVelType;
        /** \brief info required for comping up with an insertion position. */
        InsertionInfo m_positionInsertionInfo;
        /** \brief info required for comping up with an insertion orientation.*/
        InsertionInfo m_orientationInsertionInfo;
        /** \brief info required for comping up with an insertion velocity. */
        InsertionInfo m_translationalVelInsertionInfo;
        /** \brief info required for comping up with an insertion omega. */
        InsertionInfo m_angularVelInsertionInfo;
        /** \brief number of components to insert. */
        unsigned int m_numToInsert;
        //@}


	public:
		/**@name Contructors */
		//@{
        /** @brief Default constructor */
        __HOST__ 
        Insertion();

        /** @brief Constructor with XML node
		@param dn XML node */
		__HOST__
		Insertion( DOMNode* dn );

        /** @brief Constructor with InsertionType and InsertionInfo for each 
        component. 
        // TODO: params comments */
        __HOST__ 
        Insertion( InsertionType const pos,
                   InsertionType const ori,
                   InsertionType const vel,
                   InsertionType const ome,
                   InsertionInfo const& posData,
                   InsertionInfo const& oriData,
                   InsertionInfo const& velData,
                   InsertionInfo const& omeData,
                   unsigned int numToInsert );

		/** @brief Destructor */
		__HOST__
		~Insertion();
		//@}


		/** @name Get methods */
        //@{
        // TODO: INSERTION MODE AND TYPE?
        //@}


		/** @name Methods */
		//@{
        /** @brief return all required data members to insert components as a 
        vector */
        __HOST__
        std::vector<std::pair<Transform<T>, Kinematics<T>>> 
        fetchInsertionData() const;
		//@}
};

#endif
