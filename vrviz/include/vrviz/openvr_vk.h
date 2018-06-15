//========= Copyright Valve Corporation ============//

#if defined( _WIN32 )
	#define VK_USE_PLATFORM_WIN32_KHR
#else
	#define SDL_VIDEO_DRIVER_X11
	#define VK_USE_PLATFORM_XLIB_KHR
#endif
#include <vulkan/vulkan.h>
#include <SDL.h>
#include <SDL_syswm.h>
#include <stdio.h>
#include <string>
#include <cstdlib>
#include <inttypes.h>
#include <openvr.h>
#include <deque>

#include "shared/lodepng.h"
#include "shared/Matrices.h"
#include "shared/pathtools.h"

#if defined(POSIX)
#include "unistd.h"
#endif

#ifndef _countof
#define _countof(x) (sizeof(x)/sizeof((x)[0]))
#endif

void inline ThreadSleep( unsigned long nMilliseconds )
{
#if defined(_WIN32)
	::Sleep( nMilliseconds );
#elif defined(POSIX)
	usleep( nMilliseconds * 1000 );
#endif
}

// Pipeline state objects
enum PipelineStateObjectEnum_t
{
	PSO_SCENE = 0,
	PSO_AXES,
	PSO_RENDERMODEL,
	PSO_COMPANION,
	PSO_COUNT
};

// Indices of descriptor sets for rendering
enum DescriptorSetIndex_t
{
	DESCRIPTOR_SET_LEFT_EYE_SCENE = 0,
	DESCRIPTOR_SET_RIGHT_EYE_SCENE,
	DESCRIPTOR_SET_COMPANION_LEFT_TEXTURE,
	DESCRIPTOR_SET_COMPANION_RIGHT_TEXTURE,
	DESCRIPTOR_SET_LEFT_EYE_RENDER_MODEL0,
	DESCRIPTOR_SET_LEFT_EYE_RENDER_MODEL_MAX = DESCRIPTOR_SET_LEFT_EYE_RENDER_MODEL0 + vr::k_unMaxTrackedDeviceCount,
	DESCRIPTOR_SET_RIGHT_EYE_RENDER_MODEL0,
	DESCRIPTOR_SET_RIGHT_EYE_RENDER_MODEL_MAX = DESCRIPTOR_SET_RIGHT_EYE_RENDER_MODEL0 + vr::k_unMaxTrackedDeviceCount,
	NUM_DESCRIPTOR_SETS
};

class VulkanRenderModel
{
public:
	VulkanRenderModel( const std::string & sRenderModelName );
	~VulkanRenderModel();

	bool BInit( VkDevice pDevice, const VkPhysicalDeviceMemoryProperties &memoryProperties, VkCommandBuffer pCommandBuffer, vr::TrackedDeviceIndex_t unTrackedDeviceIndex, VkDescriptorSet pDescriptorSets[ 2 ], const vr::RenderModel_t & vrModel, const vr::RenderModel_TextureMap_t & vrDiffuseTexture );
	void Cleanup();
	void Draw( vr::EVREye nEye, VkCommandBuffer pCommandBuffer, VkPipelineLayout pPipelineLayout, const Matrix4 &matMVP );
	const std::string & GetName() const { return m_sModelName; }

private:
	VkDevice m_pDevice;
	VkPhysicalDeviceMemoryProperties m_physicalDeviceMemoryProperties;
	VkBuffer m_pVertexBuffer;
	VkDeviceMemory m_pVertexBufferMemory;
	VkBuffer m_pIndexBuffer;
	VkDeviceMemory m_pIndexBufferMemory;
	VkImage m_pImage;
	VkDeviceMemory m_pImageMemory;
	VkImageView m_pImageView;
	VkBuffer m_pImageStagingBuffer;
	VkDeviceMemory m_pImageStagingBufferMemory;
	VkBuffer m_pConstantBuffer[ 2 ];
	VkDeviceMemory m_pConstantBufferMemory[ 2 ];
	void *m_pConstantBufferData[ 2 ];
	VkDescriptorSet m_pDescriptorSets[ 2 ];
	VkSampler m_pSampler;

	size_t m_unVertexCount;
	vr::TrackedDeviceIndex_t m_unTrackedDeviceIndex;
	std::string m_sModelName;
};

static bool g_bPrintf = true;

// Vulkan extension entrypoints
static PFN_vkCreateDebugReportCallbackEXT g_pVkCreateDebugReportCallbackEXT = nullptr;
static PFN_vkDestroyDebugReportCallbackEXT g_pVkDestroyDebugReportCallbackEXT = nullptr;

//-----------------------------------------------------------------------------
// Purpose:
//------------------------------------------------------------------------------
class CMainApplication
{
public:
	CMainApplication( int argc, char *argv[] );
	virtual ~CMainApplication();

	bool BInit();
    virtual bool BInitVulkan();
	bool BInitVulkanInstance();
	bool BInitVulkanDevice();
	bool BInitVulkanSwapchain();
	bool BInitCompositor();
	bool GetVulkanInstanceExtensionsRequired( std::vector< std::string > &outInstanceExtensionList );
	bool GetVulkanDeviceExtensionsRequired( VkPhysicalDevice pPhysicalDevice, std::vector< std::string > &outDeviceExtensionList );

	void SetupRenderModels();

	void Shutdown();

	void RunMainLoop();
	bool HandleInput();
	void ProcessVREvent( const vr::VREvent_t & event );
	void RenderFrame();

    virtual bool SetupTexturemaps();
	static void GenMipMapRGBA( const uint8_t *pSrc, uint8_t *ppDst, int nSrcWidth, int nSrcHeight, int *pDstWidthOut, int *pDstHeightOut );

	void SetupScene();
	void AddCubeToScene( Matrix4 mat, std::vector<float> &vertdata );
	void AddCubeVertex( float fl0, float fl1, float fl2, float fl3, float fl4, std::vector<float> &vertdata );

    virtual void RenderControllerAxes();

	bool SetupStereoRenderTargets();
	void SetupCompanionWindow();
	void SetupCameras();

	void RenderStereoTargets();
	void RenderCompanionWindow();
	void RenderScene( vr::Hmd_Eye nEye );

	Matrix4 GetHMDMatrixProjectionEye( vr::Hmd_Eye nEye );
	Matrix4 GetHMDMatrixPoseEye( vr::Hmd_Eye nEye );
	Matrix4 GetCurrentViewProjectionMatrix( vr::Hmd_Eye nEye );
    virtual void UpdateHMDMatrixPose();

	Matrix4 ConvertSteamVRMatrixToMatrix4( const vr::HmdMatrix34_t &matPose );

	bool CreateAllShaders();
	void CreateAllDescriptorSets();

	void SetupRenderModelForTrackedDevice( vr::TrackedDeviceIndex_t unTrackedDeviceIndex );
	VulkanRenderModel *FindOrLoadRenderModel( vr::TrackedDeviceIndex_t unTrackedDeviceIndex, const char *pchRenderModelName );

    unsigned int m_unPointSize;
    std::string m_strTextPath;
protected:
	bool m_bDebugVulkan;
	bool m_bVerbose;
	bool m_bPerf;
	bool m_bVblank;
	int m_nMSAASampleCount;
	// Optional scaling factor to render with supersampling (defaults off, use -scale)
	float m_flSuperSampleScale;
	
	vr::IVRSystem *m_pHMD;
	vr::IVRRenderModels *m_pRenderModels;
	std::string m_strDriver;
	std::string m_strDisplay;
	vr::TrackedDevicePose_t m_rTrackedDevicePose[ vr::k_unMaxTrackedDeviceCount ];
	Matrix4 m_rmat4DevicePose[ vr::k_unMaxTrackedDeviceCount ];
	bool m_rbShowTrackedDevice[ vr::k_unMaxTrackedDeviceCount ];

// SDL bookkeeping
	SDL_Window *m_pCompanionWindow;
	uint32_t m_nCompanionWindowWidth;
	uint32_t m_nCompanionWindowHeight;


// Vulkan bookkeeping
	int m_iTrackedControllerCount;
	int m_iTrackedControllerCount_Last;
	int m_iValidPoseCount;
	int m_iValidPoseCount_Last;
	bool m_bShowCubes;

	std::string m_strPoseClasses;                            // what classes we saw poses for this frame
	char m_rDevClassChar[ vr::k_unMaxTrackedDeviceCount ];   // for each device, a character representing its class

	int m_iSceneVolumeWidth;
	int m_iSceneVolumeHeight;
	int m_iSceneVolumeDepth;
	float m_fScaleSpacing;
	float m_fScale;
	
	int m_iSceneVolumeInit;                                  // if you want something other than the default 20x20x20
	
	float m_fNearClip;
	float m_fFarClip;

	unsigned int m_uiVertcount;
	unsigned int m_uiCompanionWindowIndexSize;

	VkInstance m_pInstance;
	VkDevice m_pDevice;
	VkPhysicalDevice m_pPhysicalDevice;
	VkQueue m_pQueue;
	VkSurfaceKHR m_pSurface;
	VkSwapchainKHR m_pSwapchain;
	VkPhysicalDeviceProperties m_physicalDeviceProperties;
	VkPhysicalDeviceMemoryProperties m_physicalDeviceMemoryProperties;
	VkPhysicalDeviceFeatures m_physicalDeviceFeatures;
	uint32_t m_nQueueFamilyIndex;
	VkDebugReportCallbackEXT m_pDebugReportCallback;
	uint32_t m_nSwapQueueImageCount;
	uint32_t m_nFrameIndex;
	uint32_t m_nCurrentSwapchainImage;
	std::vector< VkImage > m_swapchainImages;
	std::vector< VkImageView > m_pSwapchainImageViews;
	std::vector< VkFramebuffer > m_pSwapchainFramebuffers;
	std::vector< VkSemaphore > m_pSwapchainSemaphores;
	VkRenderPass m_pSwapchainRenderPass;
	

	VkCommandPool m_pCommandPool;
	VkDescriptorPool m_pDescriptorPool;
	VkDescriptorSet m_pDescriptorSets[ NUM_DESCRIPTOR_SETS ];

	struct VulkanCommandBuffer_t
	{
		VkCommandBuffer m_pCommandBuffer;
		VkFence m_pFence;
	};
	std::deque< VulkanCommandBuffer_t > m_commandBuffers;
	VulkanCommandBuffer_t m_currentCommandBuffer;
	
	VulkanCommandBuffer_t GetCommandBuffer();

	// Scene resources
	VkBuffer m_pSceneVertexBuffer;
	VkDeviceMemory m_pSceneVertexBufferMemory;
	VkBufferView m_pSceneVertexBufferView;
	VkBuffer m_pSceneConstantBuffer[ 2 ];
	VkDeviceMemory m_pSceneConstantBufferMemory[ 2 ];
	void *m_pSceneConstantBufferData[ 2 ];
	VkImage m_pSceneImage;
	VkDeviceMemory m_pSceneImageMemory;
	VkImageView m_pSceneImageView;
	VkBuffer m_pSceneStagingBuffer;
	VkDeviceMemory m_pSceneStagingBufferMemory;
	VkSampler m_pSceneSampler;

	// Storage for VS and PS for each PSO
	VkShaderModule m_pShaderModules[ PSO_COUNT * 2 ];
	VkPipeline m_pPipelines[ PSO_COUNT ];
	VkDescriptorSetLayout m_pDescriptorSetLayout;
	VkPipelineLayout m_pPipelineLayout;
	VkPipelineCache m_pPipelineCache;

	// Companion window resources
	VkBuffer m_pCompanionWindowVertexBuffer;
	VkDeviceMemory m_pCompanionWindowVertexBufferMemory;
	VkBuffer m_pCompanionWindowIndexBuffer;
	VkDeviceMemory m_pCompanionWindowIndexBufferMemory;

	// Controller axes resources
	VkBuffer m_pControllerAxesVertexBuffer;
	VkDeviceMemory m_pControllerAxesVertexBufferMemory;

	unsigned int m_uiControllerVertcount;

	Matrix4 m_mat4HMDPose;
	Matrix4 m_mat4eyePosLeft;
	Matrix4 m_mat4eyePosRight;

	Matrix4 m_mat4ProjectionCenter;
	Matrix4 m_mat4ProjectionLeft;
	Matrix4 m_mat4ProjectionRight;

	struct VertexDataScene
	{
		Vector3 position;
		Vector2 texCoord;
	};

	struct VertexDataWindow
	{
		Vector2 position;
		Vector2 texCoord;

		VertexDataWindow( const Vector2 & pos, const Vector2 tex ) :  position(pos), texCoord(tex) {	}
	};

	struct FramebufferDesc
	{
		VkImage m_pImage;
		VkImageLayout m_nImageLayout;
		VkDeviceMemory m_pDeviceMemory;
		VkImageView m_pImageView;
		VkImage m_pDepthStencilImage;
		VkImageLayout m_nDepthStencilImageLayout;
		VkDeviceMemory m_pDepthStencilDeviceMemory;
		VkImageView m_pDepthStencilImageView;
		VkRenderPass m_pRenderPass;
		VkFramebuffer m_pFramebuffer;
	};
	FramebufferDesc m_leftEyeDesc;
	FramebufferDesc m_rightEyeDesc;

	bool CreateFrameBuffer( int nWidth, int nHeight, FramebufferDesc &framebufferDesc );
	
	uint32_t m_nRenderWidth;
	uint32_t m_nRenderHeight;

	std::vector< VulkanRenderModel * > m_vecRenderModels;
	VulkanRenderModel *m_rTrackedDeviceToRenderModel[ vr::k_unMaxTrackedDeviceCount ];
};

//-----------------------------------------------------------------------------
// Purpose: Outputs a set of optional arguments to debugging output, using
//          the printf format setting specified in fmt*.
//-----------------------------------------------------------------------------
void inline dprintf( const char *fmt, ... )
{
	va_list args;
	char buffer[ 2048 ];

	va_start( args, fmt );
	vsprintf_s( buffer, fmt, args );
	va_end( args );

	if ( g_bPrintf )
		printf( "%s", buffer );

	OutputDebugStringA( buffer );
}

//-----------------------------------------------------------------------------
// Purpose: VK_EXT_debug_report callback
//-----------------------------------------------------------------------------
static VkBool32 VKAPI_PTR inline VKDebugMessageCallback( VkDebugReportFlagsEXT flags, VkDebugReportObjectTypeEXT objectType, uint64_t object,
    size_t location, int32_t messageCode, const char* pLayerPrefix, const char *pMessage, void *pUserData )
{
	char buf[4096] = { 0 };
	switch ( flags )
	{
	case VK_DEBUG_REPORT_ERROR_BIT_EXT:
		sprintf( buf, "VK ERROR %s %" PRIu64 ":%d: %s\n", pLayerPrefix, uint64_t( location ), messageCode, pMessage );
		break;
	case VK_DEBUG_REPORT_WARNING_BIT_EXT:
		sprintf( buf, "VK WARNING %s %" PRIu64 ":%d: %s\n", pLayerPrefix, uint64_t( location ), messageCode, pMessage );
		break;
	case VK_DEBUG_REPORT_PERFORMANCE_WARNING_BIT_EXT:
		sprintf( buf, "VK PERF %s %" PRIu64 ":%d: %s\n", pLayerPrefix, uint64_t( location ), messageCode, pMessage );
		break;
	case VK_DEBUG_REPORT_INFORMATION_BIT_EXT:
		sprintf( buf, "VK INFO %s %" PRIu64 ":%d: %s\n", pLayerPrefix, uint64_t( location ), messageCode, pMessage );
		break;
	case VK_DEBUG_REPORT_DEBUG_BIT_EXT:
		sprintf( buf, "VK DEBUG %s %" PRIu64 ":%d: %s\n", pLayerPrefix, uint64_t( location ), messageCode, pMessage );
		break;
	default:
		break;
	}

	dprintf( "%s\n", buf );

	return VK_FALSE;
}

//-----------------------------------------------------------------------------
// Purpose: Determine the memory type index from the memory requirements
// and type bits
//-----------------------------------------------------------------------------
static bool inline MemoryTypeFromProperties( const VkPhysicalDeviceMemoryProperties &memoryProperties, uint32_t nMemoryTypeBits, VkMemoryPropertyFlags nMemoryProperties, uint32_t *pTypeIndexOut )
{
	for ( uint32_t i = 0; i < VK_MAX_MEMORY_TYPES; i++ ) 
	{
		if ( ( nMemoryTypeBits & 1 ) == 1) 
		{
			// Type is available, does it match user properties?
			if ( ( memoryProperties.memoryTypes[i].propertyFlags & nMemoryProperties ) == nMemoryProperties ) 
			{
				*pTypeIndexOut = i;
				return true;
			}
		}
		nMemoryTypeBits >>= 1;
	}

	// No memory types matched, return failure
	return false;
}

//-----------------------------------------------------------------------------
// Purpose: Helper function to create Vulkan static VB/IBs
//-----------------------------------------------------------------------------
static bool inline CreateVulkanBuffer( VkDevice pDevice, const VkPhysicalDeviceMemoryProperties &memoryProperties, const void *pBufferData, VkDeviceSize nSize, VkBufferUsageFlags nUsage, VkBuffer *ppBufferOut, VkDeviceMemory *ppDeviceMemoryOut )
{
	// Create the vertex buffer and fill with data
	VkBufferCreateInfo bufferCreateInfo = { VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
	bufferCreateInfo.size = nSize;
	bufferCreateInfo.usage = nUsage;
	VkResult nResult = vkCreateBuffer( pDevice, &bufferCreateInfo, nullptr, ppBufferOut );
	if ( nResult != VK_SUCCESS )
	{
		dprintf( "%s - vkCreateBuffer failed with error %d\n", __FUNCTION__, nResult );
		return false;
	}

	VkMemoryRequirements memoryRequirements = {};
	vkGetBufferMemoryRequirements( pDevice, *ppBufferOut, &memoryRequirements );

	VkMemoryAllocateInfo allocInfo = { VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO };
	if ( !MemoryTypeFromProperties( memoryProperties, memoryRequirements.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT, &allocInfo.memoryTypeIndex ) )
	{
		dprintf( "%s - failed to find matching memoryTypeIndex for buffer\n", __FUNCTION__ );
		return false;
	}
	allocInfo.allocationSize = memoryRequirements.size;

	nResult = vkAllocateMemory( pDevice, &allocInfo, nullptr, ppDeviceMemoryOut );
	if ( nResult != VK_SUCCESS )
	{
		dprintf( "%s - vkCreateBuffer failed with error %d\n", __FUNCTION__, nResult );
		return false;
	}

	nResult = vkBindBufferMemory( pDevice, *ppBufferOut, *ppDeviceMemoryOut, 0 );
	if ( nResult != VK_SUCCESS )
	{
		dprintf( "%s vkBindBufferMemory failed with error %d\n", __FUNCTION__, nResult );
		return false;
	}

	if ( pBufferData != nullptr )
	{
		void *pData;
		nResult = vkMapMemory( pDevice, *ppDeviceMemoryOut, 0, VK_WHOLE_SIZE, 0, &pData );
		if ( nResult != VK_SUCCESS )
		{
			dprintf( "%s - vkMapMemory returned error %d\n", __FUNCTION__, nResult );
			return false;
		}
		memcpy( pData, pBufferData, nSize );
		vkUnmapMemory( pDevice, *ppDeviceMemoryOut );

		VkMappedMemoryRange memoryRange = { VK_STRUCTURE_TYPE_MAPPED_MEMORY_RANGE };
		memoryRange.memory = *ppDeviceMemoryOut;
		memoryRange.size = VK_WHOLE_SIZE;
		vkFlushMappedMemoryRanges( pDevice, 1, &memoryRange );

	}
	return true;
}
